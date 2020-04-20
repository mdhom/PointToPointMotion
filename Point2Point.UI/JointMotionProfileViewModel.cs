using System;
using System.Collections.Generic;
using System.Windows.Input;
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

        public ICommand RandomCommand { get; }

        public JointMotionProfileViewModel()
        {
            Update(new ConstraintsCollection(
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(500, 1000, 200)));

            RandomCommand = new RelayCommand(() =>
            {
                var random = new Random((int)DateTime.Now.Ticks);

                var segments = new List<VelocityConstraint>() { new VelocityConstraint(0, random.NextDouble(200, 1000), random.Next(100, 1000)) };
                for (var i = 0; i < 15; i++)
                {
                    segments.Add(new VelocityConstraint(random.NextDouble(0, 5000), random.NextDouble(200, 1000), random.Next(100, 1000)));
                }
                var joint = new ConstraintsCollection(segments);
                Update(joint);
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

            DrawRawConstraints(constraintsCollection, plotModel);
            
            DrawEffectiveConstraints(constraintsCollection, plotModel);
            
            DrawJointMotionProfile(constraintsCollection, plotModel);

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

        private static void DrawJointMotionProfile(ConstraintsCollection constraintsCollection, PlotModel plotModel)
        {
            var jointProfile = new JointMotionProfile(constraintsCollection, new P2PParameters(2000, 500, 1000));
            var profilePoints = jointProfile.CalculateProfile();

            var jointSerie = new LineSeries()
            {
                Title = "JointProfile",
                Color = OxyColors.Black,
                ItemsSource = new List<DataPoint>()
            };

            foreach (var point in profilePoints)
            {
                (jointSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(point.Distance, point.Velocity));
            }

            plotModel.Series.Add(jointSerie);
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
