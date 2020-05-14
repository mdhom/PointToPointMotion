namespace Point2Point.JointMotion
{
    public class VelocityPoint
    {
        public double Distance { get; }
        public double Acceleration { get; }
        public double Velocity { get; }
        public VelocityConstraint CorrespondingConstraint { get; }

        public VelocityPoint(double distance, double acceleration, double velocity, VelocityConstraint correspondingConstraint)
        {
            Distance = distance;
            Acceleration = acceleration;
            Velocity = velocity;
            CorrespondingConstraint = correspondingConstraint;
        }

        public VelocityPoint(double distance, double velocity, VelocityConstraint correspondingConstraint)
            : this(distance, 0.0, velocity, correspondingConstraint)
        {
        }
    }
}
