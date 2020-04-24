namespace Shuttles.Base.Devices.Shuttles.Motion.Ramp
{
    public struct RampCalculationResult
    {
        public bool IsFlat { get; set; }

        public RampMotionParameter Parameters { get; set; }

        public double vFrom { get; set; }
        public double vTo { get; set; }

        public double Phase1Duration { get; set; }
        public double Phase2Duration { get; set; }
        public double Phase3Duration { get; set; }

        public double TotalDistance { get; set; }
        public double TotalDuration { get; set; }

        public RampDirection Direction { get; set; }

        public bool IsReachable(double availableDistance)
            => TotalDistance <= availableDistance;
    }
}
