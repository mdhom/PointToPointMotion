using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;
using OxyPlot;
using Point2Point.JointMotion;

namespace Point2Point.UI
{
    public class JointMotionProfileViewModel : NotifyPropertyChangedBase
    {
        public List<DataPoint> DataRaw0 { get; private set; } = new List<DataPoint>();
        public List<DataPoint> DataRaw1 { get; private set; } = new List<DataPoint>();
        public List<DataPoint> DataRaw2 { get; private set; } = new List<DataPoint>();
        public List<DataPoint> DataEffective { get; private set; } = new List<DataPoint>();

        public ICommand RandomCommand { get; }

        public JointMotionProfileViewModel()
        {
            var joint = new JointMotionProfile(
                new MotionProfileSegment(0, 1000, 500),
                new MotionProfileSegment(1000, 1000, 400),
                new MotionProfileSegment(500, 1000, 200));

            var effectiveSegments = joint.GetEffectiveSegments();


            DataRaw0.Add(new DataPoint(0, 500));
            DataRaw0.Add(new DataPoint(1000, 500));
            DataRaw1.Add(new DataPoint(1000, 400));
            DataRaw1.Add(new DataPoint(2000, 400));
            DataRaw2.Add(new DataPoint(500, 200));
            DataRaw2.Add(new DataPoint(1500, 200));

            foreach (var segment in effectiveSegments)
            {
                DataEffective.Add(new DataPoint(segment.Start, segment.MaximumVelocity));
                DataEffective.Add(new DataPoint(segment.End, segment.MaximumVelocity));
            }
        }
    }
}
