namespace Point2Point.JointMotion
{
    public struct VelocityPoint
    {
        public double Distance { get; }
        public double Velocity { get; }

        public VelocityPoint(double distance, double velocity)
        {
            Distance = distance;
            Velocity = velocity;
        }
    }
}
