using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows.Data;
using System.Windows.Input;
using Microsoft.Win32;
using Newtonsoft.Json;
using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;
using Point2Point.JointMotion;
using Servus.Core.Ui;

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

        public ICommand RandomCommand { get; }
        public ICommand RecalcCommand { get; }
        public ICommand SaveCommand { get; }
        public ICommand LoadCommand { get; }

        private ConstraintsCollection _randomConstraints;

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

            RecalcCommand = new RelayCommand(() =>
            {
                if (_randomConstraints != null)
                {
                    Update(_randomConstraints);
                }
            });

            SaveCommand = new RelayCommand(() =>
            {
                if (_randomConstraints != null)
                {
                    File.WriteAllText($"{DateTime.Now:yyyy-MM-dd-hh-mm-ss}.json", JsonConvert.SerializeObject(_randomConstraints));
                }
            });

            LoadCommand = new RelayCommand(() =>
            {
                var dialog = new OpenFileDialog();
                if (dialog.ShowDialog().GetValueOrDefault(false))
                {
                    var jsonContent = File.ReadAllText(dialog.FileName);
                    _randomConstraints = JsonConvert.DeserializeObject<ConstraintsCollection>(jsonContent);
                    Update(_randomConstraints);
                }
            });
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

            var profile = new JointMotionProfile(constraintsCollection, new P2PParameters(2000, 500, 1000));

            if (DrawVelocityPoints)
            {
                DrawVelocityPointsProfile(profile, plotModel);
            }

            if (DrawMotionProfile)
            {
                DrawJointMotionProfile(profile, plotModel);
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

            foreach (var point in jointMotionProfile.VelocityPoints)
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

        private static void DrawJointMotionProfile(JointMotionProfile jointMotionProfile, PlotModel plotModel)
        {
            try
            {
                var jointSerie = new LineSeries()
                {
                    Title = "Profile",
                    Color = OxyColors.Gray,
                    ItemsSource = new List<DataPoint>()
                };

                for (double t = 0; t < jointMotionProfile.TotalDuration; t += 0.01)
                {
                    try
                    {
                        jointMotionProfile.GetStatus(t, out var v, out var s);
                        (jointSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(s, v));
                    }
                    catch (Exception ex)
                    {
                        break;
                    }
                }

                plotModel.Series.Add(jointSerie);
            }
            catch (Exception ex)
            {
            }
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
