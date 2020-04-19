using Point2Point.Calculation;

namespace Shuttles.Base.Devices.Shuttles.Motion.Ramp
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
