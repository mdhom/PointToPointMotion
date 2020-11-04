using System;
using System.Runtime.CompilerServices;

[assembly:InternalsVisibleTo("Point2Point.Tests")]

namespace Point2Point
{
    internal static class MathematicTools
    {
        private static readonly double _third = 1.0 / 3;

        public static double GetCubeRoot(double x)
        {
            return Math.Pow(x, _third);
        }

        public static bool SolveEquation(double a, double b, double c, out double x1, out double x2)
        {
            if (a == 0 && b == 0)
            {
                throw new ArgumentOutOfRangeException(nameof(a), $"a and b must not be zero together, because it is no equation anymore");
            }

            x1 = 0;
            x2 = 0;
            if (a == 0)
            {
                // linear equation bx+c=0
                x1 = -c / b;
                x2 = x1;
                return true;
            }
            else
            {
                // quadratic equation ax²+bx+c=0
                // solve with "Mitternachtsformel"
                var determinant = b * b - 4 * a * c;
                if (determinant < 0)
                {
                    return false;
                }

                var root = Math.Sqrt(determinant);
                x1 = (-b + root) / (2 * a);
                x2 = (-b - root) / (2 * a);
                return true;
            }
        }
    }
}
