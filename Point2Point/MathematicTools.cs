namespace System
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
            x1 = 0;
            x2 = 0;
            if (a == 0)
            {
                // it is no quadratic equation!
                x1 = -c / b;
                x2 = x1;
                return true;
            }
            else
            {
                // solve quadratic equation with "Mitternachtsformel"
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
