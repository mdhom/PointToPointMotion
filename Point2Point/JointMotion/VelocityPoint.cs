namespace Point2Point.JointMotion
{
    public struct VelocityPoint
    {
        public double Distance { get; }
        public double Velocity { get; }
        public VelocityConstraint CorrespondingConstraint { get; }

        public VelocityPoint(double distance, double velocity, VelocityConstraint correspondingConstraint)
        {
            Distance = distance;
            Velocity = velocity;
            CorrespondingConstraint = correspondingConstraint;
        }
    }
}
