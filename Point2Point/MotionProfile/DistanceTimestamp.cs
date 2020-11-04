namespace Point2Point.JointMotion
{
    public struct DistanceTimestamp
    {
        public double Distance { get; set; }
        public double Time { get; set; }

        public DistanceTimestamp(double distance, double time)
        {
            Distance = distance;
            Time = time;
        }
    }
}
