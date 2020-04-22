namespace Point2Point.JointMotion
{
    public class VelocityPoint
    {
        public double Distance { get; set; }
        public double Velocity { get; set; }
        public VelocityConstraint CorrespondingConstraint { get; set; }

        public VelocityPoint(double distance, double velocity, VelocityConstraint correspondingConstraint)
        {
            Distance = distance;
            Velocity = velocity;
            CorrespondingConstraint = correspondingConstraint;
        }
    }
}
