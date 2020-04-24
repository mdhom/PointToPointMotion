using Shuttles.Base.Devices.Shuttles.Motion.Ramp;

namespace Point2Point.JointMotion
{
    public class ExtendedRampCalculationResult : RampCalculationResult
    {
        public double StartDistance { get; set; }
        public double EndDistance => StartDistance + Length;

        public ExtendedRampCalculationResult(RampCalculationResult result, double startDistance)
            : base(result)
        {
            StartDistance = startDistance;
        }
    }
}
