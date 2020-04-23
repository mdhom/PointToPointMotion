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
        private PlotModel _plotModel;
        public PlotModel PlotModel
        {
            get => _plotModel;
            set => ChangeProperty(value, ref _plotModel);
        }

        private bool _drawRawSeries;
        public bool DrawRawSeries
        {
            get => _drawRawSeries;
            set => ChangeProperty(value, ref _drawRawSeries, () => Update(_randomConstraints));
        }

        private bool _drawMotionProfile;
        public bool DrawMotionProfile
        {
            get => _drawMotionProfile;
            set => ChangeProperty(value, ref _drawMotionProfile, () => Update(_randomConstraints));
        }

        private bool _drawVelocityPoints = true;
        public bool DrawVelocityPoints
        {
            get => _drawVelocityPoints;
            set => ChangeProperty(value, ref _drawVelocityPoints, () => Update(_randomConstraints));
        }

        private int _historyNavigationIndex = -1;
        public int HistoryNavigationIndex
        {
            get => _historyNavigationIndex;
            set => ChangeProperty(value, ref _historyNavigationIndex);
        }

        public bool RandomTestRunning { get; private set; }

        public ICommand RandomCommand { get; }
        public ICommand RandomTestCommand { get; }
        public ICommand RecalcCommand { get; }
        public ICommand SaveCommand { get; }
        public ICommand LoadCommand { get; }
        public ICommand NavigateHistoryCommand { get; }
        public ICommand StepCommand { get; }

        private ConstraintsCollection _randomConstraints;
        private readonly SemaphoreSlim _stepSemaphore = new SemaphoreSlim(0, 1);
        private Task _randomTestTask;
        private CancellationTokenSource _randomTestCancellationTokenSource = new CancellationTokenSource();

        public JointMotionProfileViewModel()
        {
            _randomConstraints = new ConstraintsCollection(
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(500, 1000, 200));
            Update(_randomConstraints);

            RandomCommand = new RelayCommand(() =>
            {
                var random = new Random((int)DateTime.Now.Ticks);

                var segments = new List<VelocityConstraint>() { new VelocityConstraint(0, random.NextDouble(200, 1000), random.Next(100, 1000)) };
                for (var i = 0; i < 15; i++)
                {
                    segments.Add(new VelocityConstraint(random.NextDouble(0, 5000), random.NextDouble(200, 1000), random.Next(100, 1000)));
                }
                _randomConstraints = new ConstraintsCollection(segments);
                Update(_randomConstraints);
            });

            RandomTestCommand = new RelayCommand(() =>
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
                                RandomCommand.Execute(null);
                                await Task.Delay(500);
                            }
                        }
                        finally
                        {
                            RandomTestRunning = false;
                        }
                    });
                }
            });

            RecalcCommand = new RelayCommand(() =>
            {
                if (_randomConstraints != null)
                {
                    HistoryNavigationIndex = -1;
                    Update(_randomConstraints);
                }
            });

            SaveCommand = new RelayCommand(() =>
            {
                if (_randomConstraints != null)
                {
                    SaveConstraintsCollection(_randomConstraints);
                }
            });

            LoadCommand = new RelayCommand(() =>
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
            });

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
            });
        }

        private string SaveConstraintsCollection(ConstraintsCollection constraintsCollection)
        {
            var filename = $"{DateTime.Now:yyyy-MM-dd-HH-mm-ss}.json";
            File.WriteAllText(filename, JsonConvert.SerializeObject(constraintsCollection));
            return filename;
        }

        private void Update(ConstraintsCollection constraintsCollection)
        {
            var plotModel = new PlotModel()
            {
                Title = "Joint motion"
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

                if (profile.VelocityProfilePoints != null && DrawVelocityPoints)
                {
                    DrawVelocityPointsProfile(profile, plotModel);
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
                File.WriteAllText(exceptionFilename, sb.ToString());
            }

            PlotModel = plotModel;
        }

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


            var effSerie = new LineSeries()
            {
                Title = "JointProfile Edited",
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
    }
}
