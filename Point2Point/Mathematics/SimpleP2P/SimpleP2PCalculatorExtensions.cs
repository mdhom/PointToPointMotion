using Point2Point.Mathematics.ExtendedP2P;
using Point2Point.Mathematics.SimpleP2P;

namespace Point2Point   // using "wrong" namespace here for making extensions avaiable at ease
{
    public static class SimpleP2PCalculatorExtensions
    {
        public static double GetBrakingDistance(this SimpleP2PCalculator calculator, double t)
        {
            calculator.GetStatus(t, out _, out var a, out var v, out _);
            var motionParameter = new MotionParameter(calculator.JerkMax, -calculator.JerkMax, calculator.AccelerationMax, -calculator.AccelerationMax);
            var rampResult = ExtendedP2PCalculator.Calculate(a, v, 0, motionParameter);
            return rampResult.Length;
        }
    }
}
