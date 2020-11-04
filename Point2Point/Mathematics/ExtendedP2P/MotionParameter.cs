using Point2Point.Mathematics.SimpleP2P;

namespace Point2Point.Mathematics.ExtendedP2P
{
    public struct MotionParameter
    {
        public double PositiveJerk { get; set; }
        public double NegativeJerk { get; set; }
        public double MaximumAcceleration { get; set; }
        public double MaximumDecceleration { get; set; }

        public MotionParameter(double positiveJerk, double negativeJerk, double maximumAcceleration, double maximumDecceleration)
        {
            PositiveJerk = positiveJerk;
            NegativeJerk = negativeJerk;
            MaximumAcceleration = maximumAcceleration;
            MaximumDecceleration = maximumDecceleration;
        }

        public MotionParameter(SimpleP2PParameters parameters)
        {
            PositiveJerk = parameters.JerkMax;
            NegativeJerk = -parameters.JerkMax;
            MaximumAcceleration = parameters.AccelerationMax;
            MaximumDecceleration = -parameters.AccelerationMax;
        }
    }
}
