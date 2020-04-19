namespace Shuttles.Base.Devices.Shuttles.Motion.Ramp
{
    public class RampCalculationResult
    {
        public double t1 { get; set; }
        public double t2 { get; set; }
        public double t3 { get; set; }

        public double TotalDistance { get; set; }

        public double TotalDuration { get; set; }

        public double AccDeccReached_Square { get; set; }
        public double AccDeccReached { get; set; }

        public RampMotionState MotionState { get; set; }

        public bool InvertedAccDeccState { get; set; }

        public void Invert()
        {
            InvertedAccDeccState = !InvertedAccDeccState;
        }
    }
}
