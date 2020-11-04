namespace Point2Point.Mathematics.ExtendedP2P
{
    public class ExtendedP2PCalculatorResult
    {
        public MotionParameter Parameters { get; set; }
        public RampDirection Direction { get; set; }

        public double aFrom { get; set; }
        public double vFrom { get; set; }
        public double vTo { get; set; }

        public double Phase1Duration { get; set; }
        public double Phase1Length { get; set; }
        public double Phase2Duration { get; set; }
        public double Phase2Length { get; set; }
        public double Phase3Duration { get; set; }
        public double Phase3Length { get; set; }

        public double Length { get; set; }
        public double TotalDuration { get; set; }

        public bool IsReachable(double availableDistance)
            => Length <= availableDistance;

        public ExtendedP2PCalculatorResult()
        {
        }

        public ExtendedP2PCalculatorResult(ExtendedP2PCalculatorResult result)
        {
            Parameters = result.Parameters;
            Direction = result.Direction;
            aFrom = result.aFrom;
            vFrom = result.vFrom;
            vTo = result.vTo;
            Phase1Duration = result.Phase1Duration;
            Phase2Duration = result.Phase2Duration;
            Phase3Duration = result.Phase3Duration;
            Phase1Length = result.Phase1Length;
            Phase2Length = result.Phase2Length;
            Phase3Length = result.Phase3Length;
            Length = result.Length;
            TotalDuration = result.TotalDuration;
        }
    }
}
