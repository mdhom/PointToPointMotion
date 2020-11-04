using Point2Point.Mathematics.ExtendedP2P;

namespace Point2Point.JointMotion
{
    public class ExtendedRampCalculationResult : ExtendedP2PCalculatorResult
    {
        public double StartDistance { get; set; }
        public double EndDistance => StartDistance + Length;
        public double StartDuration { get; set; }

        public ExtendedRampCalculationResult(ExtendedP2PCalculatorResult result, double startDistance, double startDuration)
            : base(result)
        {
            StartDistance = startDistance;
            StartDuration = startDuration;
        }
    }
}
