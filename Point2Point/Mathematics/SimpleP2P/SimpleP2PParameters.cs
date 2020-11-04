namespace Point2Point.Mathematics.SimpleP2P
{
    public class SimpleP2PParameters
    {
        public double JerkMax { get; }
        public double AccelerationMax { get; }
        public double VelocityMax { get; }

        public SimpleP2PParameters(double jerkMax, double accelerationMax, double velocityMax)
        {
            JerkMax = jerkMax;
            AccelerationMax = accelerationMax;
            VelocityMax = velocityMax;
        }
    }
}
