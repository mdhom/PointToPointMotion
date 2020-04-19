namespace Point2Point.JointMotionProfile
{
    public class MotionProfileSegment
    {
        public double Start { get; set; }
        public double Length { get; }
        public double MaximumVelocity { get; }

        public double End => Start + Length;

        public MotionProfileSegment(double start, double length, double maximumVelocity)
        {
            Start = start;
            Length = length;
            MaximumVelocity = maximumVelocity;
        }

        public bool Contains(double distance) => Start <= distance && End > distance;
    }
}
