namespace Point2Point
{
    public class P2PParameters
    {
        public double JerkMax { get; }
        public double AccelerationMax { get; }
        public double VelocityMax { get; }

        public P2PParameters(double jerkMax, double accelerationMax, double velocityMax)
        {
            JerkMax = jerkMax;
            AccelerationMax = accelerationMax;
            VelocityMax = velocityMax;
        }
    }
}
