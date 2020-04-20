using System;

namespace Point2Point.UI
{
    public static class RandomExtensions
    {
        public static double NextDouble(this Random random, double min, double max)
        {
            var randomDouble = random.NextDouble();
            return min + randomDouble * (max - min);
        }
    }
}
