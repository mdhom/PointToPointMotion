using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
            Update(new JointMotionProfile(
                new MotionProfileSegment(0, 1000, 500),
                new MotionProfileSegment(1000, 1000, 400),
                new MotionProfileSegment(500, 1000, 200)));

            RandomCommand = new RelayCommand(() =>
            {
                var random = new Random((int)DateTime.Now.Ticks);

                var segments = new List<MotionProfileSegment>() { new MotionProfileSegment(0, random.NextDouble(200, 1000), random.Next(100, 1000)) };
                for (int i = 0; i < 15; i++)
                {
                    segments.Add(new MotionProfileSegment(random.NextDouble(0, 5000), random.NextDouble(200, 1000), random.Next(100, 1000)));
                }
                var joint = new JointMotionProfile(segments);
                Update(joint);
            });
        }

        private void Update(JointMotionProfile joint)
        {
            var effectiveSegments = joint.GetEffectiveSegments();

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

            var index = 0;
            foreach (var segment in joint.Segments)
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

            var effectiveSerie = new LineSeries()
            {
                Title = $"Effective",
                ItemsSource = new List<DataPoint>()
            };
            foreach (var segment in effectiveSegments)
            {
                (effectiveSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(segment.Start, segment.MaximumVelocity));
                (effectiveSerie.ItemsSource as List<DataPoint>).Add(new DataPoint(segment.End, segment.MaximumVelocity));
            }
            plotModel.Series.Add(effectiveSerie);

            PlotModel = plotModel;
        }
    }
}
