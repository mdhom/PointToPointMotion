using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using Microsoft.Win32;
using Newtonsoft.Json;
using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;
using Point2Point.JointMotion;
using Servus.Core.Ui;
using Shuttles.Base.Devices.Shuttles.Motion.Ramp;

namespace Point2Point.UI
{
    public class JointMotionProfileViewModel : NotifyPropertyChangedBase
    {
        private const string _logFolder = "log";

        private PlotModel _plotModel;
        public PlotModel PlotModel
        {
            get => _plotModel;
            set => ChangeProperty(value, ref _plotModel);
        }

        private PlotModel _plotModelVelocityProfile;
        public PlotModel PlotModelVelocityProfile
        {
            get => _plotModelVelocityProfile;
            set => ChangeProperty(value, ref _plotModelVelocityProfile);
        }

        private bool _drawRawSeries;
        public bool DrawRawSeries
        {
            get => _drawRawSeries;
            set => ChangeProperty(value, ref _drawRawSeries, () => Update(_randomConstraints));
        }

        private bool _drawMotionProfile = true;
        public bool DrawMotionProfile
        {
            get => _drawMotionProfile;
            set => ChangeProperty(value, ref _drawMotionProfile, () => Update(_randomConstraints));
        }

        private bool _drawVelocityPoints;
        public bool DrawVelocityPoints
        {
            get => _drawVelocityPoints;
            set => ChangeProperty(value, ref _drawVelocityPoints, () => Update(_randomConstraints));
        }

        private bool _drawModifiedConstraints;
        public bool DrawModifiedConstraints
        {
            get => _drawModifiedConstraints;
            set => ChangeProperty(value, ref _drawModifiedConstraints, () => Update(_randomConstraints));
        }

        private int _historyNavigationIndex = -1;
        public int HistoryNavigationIndex
        {
            get => _historyNavigationIndex;
            set => ChangeProperty(value, ref _historyNavigationIndex);
        }

        private int _numRecalculations;
        public int NumRecalculations
        {
            get => _numRecalculations;
            set => ChangeProperty(value, ref _numRecalculations);
        }

        private bool _randomTestRunning;
        public bool RandomTestRunning
        {
            get => _randomTestRunning;
            set => ChangeProperty(value, ref _randomTestRunning, () =>
            {
                App.Current.Dispatcher.Invoke(() =>
                {
                    RandomCommand.RaiseCanExecuteChanged();
                    RecalcCommand.RaiseCanExecuteChanged();
                    SaveCommand.RaiseCanExecuteChanged();
                    LoadCommand.RaiseCanExecuteChanged();
                    NavigateHistoryCommand.RaiseCanExecuteChanged();
                    StepCommand.RaiseCanExecuteChanged();
                });
            });
        }

        private int _numRandomTestRuns;
        public int NumRandomTestRuns
        {
            get => _numRandomTestRuns;
            set => ChangeProperty(value, ref _numRandomTestRuns);
        }

        public RelayCommand RandomCommand { get; }
        public RelayCommand RandomTestCommand { get; }
        public RelayCommand RecalcCommand { get; }
        public RelayCommand SaveCommand { get; }
        public RelayCommand LoadCommand { get; }
        public RelayCommand<int> NavigateHistoryCommand { get; }
        public RelayCommand StepCommand { get; }

        private readonly SemaphoreSlim _stepSemaphore = new SemaphoreSlim(0, 1);
        private ConstraintsCollection _randomConstraints;
        private Task _randomTestTask;
        private CancellationTokenSource _randomTestCancellationTokenSource = new CancellationTokenSource();

        public JointMotionProfileViewModel()
        {
            Directory.CreateDirectory(_logFolder);

            _randomConstraints = new ConstraintsCollection(
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(500, 1000, 200));

            Update(_randomConstraints);

            RandomCommand = new RelayCommand(() =>
            {
                GenerateRandomProfile();
            }, o => !RandomTestRunning);

            RandomTestCommand = new RelayCommand(() =>
            {
                ToggleRandomTestState();
            });

            RecalcCommand = new RelayCommand(() =>
            {
                if (_randomConstraints != null)
                {
                    HistoryNavigationIndex = -1;
                    Update(_randomConstraints);
                }
            }, o => !RandomTestRunning);

            SaveCommand = new RelayCommand(() =>
            {
                if (_randomConstraints != null)
                {
                    SaveConstraintsCollection(_randomConstraints);
                }
            }, o => !RandomTestRunning);

            LoadCommand = new RelayCommand(() =>
            {
                LoadConstraintsCollection();
            }, o => !RandomTestRunning);

            NavigateHistoryCommand = new RelayCommand<int>((d) =>
            {
                HistoryNavigationIndex += d;
                Update(_randomConstraints);
            });

            StepCommand = new RelayCommand(() =>
            {
                _stepSemaphore.Release();
                HistoryNavigationIndex++;
                Update(_randomConstraints);
            }, o => !RandomTestRunning);
        }

        #region Load / Save

        private void LoadConstraintsCollection()
        {
            var dialog = new OpenFileDialog()
            {
                Filter = "JSON File|*.json"
            };
            if (dialog.ShowDialog().GetValueOrDefault(false))
            {
                var jsonContent = File.ReadAllText(dialog.FileName);
                _randomConstraints = JsonConvert.DeserializeObject<ConstraintsCollection>(jsonContent);
                Update(_randomConstraints);
            }
        }

        private string SaveConstraintsCollection(ConstraintsCollection constraintsCollection)
        {
            var filename = Path.Combine(_logFolder, $"{DateTime.Now:yyyy-MM-dd-HH-mm-ss}.json");
            File.WriteAllText(filename, JsonConvert.SerializeObject(constraintsCollection));
            return filename;
        }

        #endregion

        private void Update(ConstraintsCollection constraintsCollection)
        {
            var plotModel = new PlotModel()
            {
                Title = "Joint motion over way [mm]"
            };

            plotModel.Axes.Add(new LinearAxis()
            {
                Position = AxisPosition.Left,
                Minimum = 0,
                AbsoluteMinimum = 0
            });

            if (DrawRawSeries)
            {
                DrawRawConstraints(constraintsCollection, plotModel);
            }

            DrawEffectiveConstraints(constraintsCollection, plotModel);

            try
            {
                var parameters = new RampMotionParameter(
                    positiveJerk: 2000,
                    negativeJerk: -2000,
                    maximumAcceleration: 500,
                    maximumDecceleration: -200);

                var profile = new JointMotionProfile(parameters, constraintsCollection);

                NumRecalculations = profile.NumRecalculations;

                if (profile.VelocityProfilePoints != null && DrawVelocityPoints)
                {
                    DrawVelocityPointsProfile(profile, plotModel);
                }

                if (DrawModifiedConstraints)
                {
                    DrawModifiedConstraintsProfile(profile, plotModel);
                }

                if (profile.TotalDuration != 0 && DrawMotionProfile)
                {
                    DrawJointMotionProfile(profile, plotModel);
                }

                if (HistoryNavigationIndex >= 0 && HistoryNavigationIndex < profile.EffectiveConstraintsHistory.Count)
                {
                    DrawEffectiveConstraintsHistory(profile.EffectiveConstraintsHistory[HistoryNavigationIndex], plotModel);
                }
            }
            catch (Exception ex)
            {
                var filename = SaveConstraintsCollection(constraintsCollection);
                var exceptionFilename = Path.GetFileNameWithoutExtension(filename) + "-exception.json";
                var sb = new StringBuilder();
                sb.Append($"{ex.GetType()} - {ex.Message} \r\n{ex.StackTrace}");
                while (ex.InnerException != null)
                {
                    ex = ex.InnerException;
                    sb.Append($"{ex.GetType()} - {ex.Message} \r\n{ex.StackTrace}");
                }
                File.WriteAllText(Path.Combine(_logFolder, exceptionFilename), sb.ToString());
            }

            PlotModel = plotModel;
        }

        #region Draw methods 

        private static void DrawRawConstraints(ConstraintsCollection constraintsCollection, PlotModel plotModel)
        {
            var index = 0;
            foreach (var segment in constraintsCollection)
            {
                var lineSerie = new LineSeries()
                {
                    StrokeThickness = 5,
                    Title = $"Segment {index}",
                    ItemsSource = new List<DataPoint>() {
                        new DataPoint(segment.Start, segment.MaximumVelocity),
                        new DataPoint(segment.End, segment.MaximumVelocity)
                    }
                };
                plotModel.Series.Add(lineSerie);
                index++;
            }
        }

        private static void DrawVelocityPointsProfile(JointMotionProfile jointMotionProfile, PlotModel plotModel)
        {
            var jointSerie = new LineSeries()
            {
                Title = "JointProfile",
                Color = OxyColors.Black,
                ItemsSource = new List<DataPoint>()
            };

            foreach (var point in jointMotionProfile.VelocityProfilePoints)
            {
                (jointSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(point.Distance, point.Velocity));
            }

            plotModel.Series.Add(jointSerie);
        }

        private static void DrawModifiedConstraintsProfile(JointMotionProfile jointMotionProfile, PlotModel plotModel)
        {
            var effSerie = new LineSeries()
            {
                Title = "JointProfile Modified",
                Color = OxyColors.Purple,
                ItemsSource = new List<DataPoint>()
            };

            foreach (var point in jointMotionProfile.EffectiveConstraints)
            {
                (effSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(point.Start, point.MaximumVelocity));
                (effSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(point.End, point.MaximumVelocity));
            }

            plotModel.Series.Add(effSerie);
        }

        private void DrawJointMotionProfile(JointMotionProfile jointMotionProfile, PlotModel plotModel)
        {
            var jointSerie = new LineSeries()
            {
                Title = "Profile",
                Color = OxyColors.Gray,
                ItemsSource = new List<DataPoint>()
            };

            var plotModelVelocityProfile = new PlotModel()
            {
                Title = "VelocityProfile over time [s]"
            };

            plotModelVelocityProfile.Axes.Add(new LinearAxis()
            {
                Position = AxisPosition.Left,
                Minimum = 0,
                AbsoluteMinimum = 0
            });
            var velocityProfileSerie = new LineSeries()
            {
                Title = "Profile",
                Color = OxyColors.Red,
                ItemsSource = new List<DataPoint>()
            };

            try
            {
                for (double t = 0; t < jointMotionProfile.TotalDuration; t += 0.01)
                {
                    jointMotionProfile.GetStatus(t, out var v, out var s);
                    if (jointMotionProfile.EffectiveConstraintsHistory.First().IsOutOfBounds(s, v))
                    {
                        throw new JointMotionCalculationException($"Out of bounds at {s}mm with {v}mm/s");
                    }

                    (jointSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(s, v));
                    (velocityProfileSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(t, v));
                }
            }
            catch (Exception ex)
            {
                if (RandomTestRunning)
                {
                    throw;
                }
            }

            plotModel.Series.Add(jointSerie);
            plotModelVelocityProfile.Series.Add(velocityProfileSerie);

            PlotModelVelocityProfile = plotModelVelocityProfile;
        }

        private static void DrawEffectiveConstraintsHistory(List<VelocityConstraint> historyEntry, PlotModel plotModel)
        {
            var effectiveSerie = new LineSeries()
            {
                Title = $"History",
                Color = OxyColors.Orange,
                ItemsSource = new List<DataPoint>()
            };

            foreach (var segment in historyEntry)
            {
                (effectiveSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(segment.Start, segment.MaximumVelocity));
                (effectiveSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(segment.End, segment.MaximumVelocity));
            }

            plotModel.Series.Add(effectiveSerie);
        }

        private static void DrawEffectiveConstraints(ConstraintsCollection constraintsCollection, PlotModel plotModel)
        {
            var effectiveSegments = constraintsCollection.GetEffectiveConstraints();

            var effectiveSerie = new LineSeries()
            {
                Title = $"Effective",
                Color = OxyColors.Red,
                ItemsSource = new List<DataPoint>()
            };

            foreach (var segment in effectiveSegments)
            {
                (effectiveSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(segment.Start, segment.MaximumVelocity));
                (effectiveSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(segment.End, segment.MaximumVelocity));
            }

            plotModel.Series.Add(effectiveSerie);
        }

        #endregion

        #region Random + RandomTest

        private void GenerateRandomProfile()
        {
            var random = new Random((int)DateTime.Now.Ticks);

            var segments = new List<VelocityConstraint>() { new VelocityConstraint(0, random.NextDouble(200, 1000), random.Next(100, 1000)) };
            for (var i = 0; i < 15; i++)
            {
                segments.Add(new VelocityConstraint(random.NextDouble(0, 5000), random.NextDouble(200, 1000), random.Next(100, 1000)));
            }
            _randomConstraints = new ConstraintsCollection(segments);
            Update(_randomConstraints);
        }

        private void ToggleRandomTestState()
        {
            if (RandomTestRunning && _randomTestTask != null)
            {
                // STOP
                _randomTestCancellationTokenSource.Cancel();
            }
            else
            {
                // START
                _randomTestCancellationTokenSource = new CancellationTokenSource();
                _randomTestTask = Task.Run(async () =>
                {
                    RandomTestRunning = true;
                    try
                    {
                        while (!_randomTestCancellationTokenSource.IsCancellationRequested)
                        {
                            GenerateRandomProfile();
                            NumRandomTestRuns++;
                            await Task.Delay(20);
                        }
                    }
                    finally
                    {
                        RandomTestRunning = false;
                    }
                });
            }
        }

        #endregion
    }
}
