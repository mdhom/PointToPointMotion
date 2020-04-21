using Point2Point;

namespace Shuttles.Base.Devices.Shuttles.Motion.Ramp
{
    public class RampMotionParameter
    {
        public double PositiveJerk { get; set; }
        public double NegativeJerk { get; set; }
        public double MaximumAcceleration { get; set; }
        public double MaximumDecceleration { get; set; }

        public RampMotionParameter(double positiveJerk, double negativeJerk, double maximumAcceleration, double maximumDecceleration)
        {
            PositiveJerk = positiveJerk;
            NegativeJerk = negativeJerk;
            MaximumAcceleration = maximumAcceleration;
            MaximumDecceleration = maximumDecceleration;
        }

        public RampMotionParameter(P2PParameters parameters)
        {
            PositiveJerk = parameters.JerkMax;
            NegativeJerk = -parameters.JerkMax;
            MaximumAcceleration = parameters.AccelerationMax;
            MaximumDecceleration = -parameters.AccelerationMax;
        }
    }
}
