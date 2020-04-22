namespace System
{
    public static class CubeRoot
    {
        private static readonly double _third = 1.0 / 3;

        public static double Get(double x)
        {
            return Math.Pow(x, _third);
        }
    }
}
