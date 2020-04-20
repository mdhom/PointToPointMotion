using Shuttles.Base.Devices.Shuttles.Motion.Ramp;

namespace Point2Point   // using "wron" namespace here for making extensions avaiable at ease
{
    public static class P2PCalculatorExtensions
    {
        public static double GetBrakingDistance(this P2PCalculator calculator, double t)
        {
            calculator.GetStatus(t, out _, out var a, out var v, out _);
            var motionParameter = new RampMotionParameter(calculator.JerkMax, -calculator.JerkMax, calculator.AccelerationMax, -calculator.AccelerationMax);
            var rampResult = RampCalculator.Calculate(a, v, motionParameter, 0);
            return rampResult.TotalDistance;
        }
    }
}
