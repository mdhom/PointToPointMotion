namespace Point2Point.JointMotion
{
    public class VelocityConstraint
    {
        public double Start { get; set; }
        public double Length { get; set; }
        public double MaximumVelocity { get; }

        public double End => Start + Length;

        public VelocityConstraint(double start, double length, double maximumVelocity)
        {
            Start = start;
            Length = length;
            MaximumVelocity = maximumVelocity;
        }

        public bool Contains(double distance) 
            => Start <= distance && End > distance;
    }
}
