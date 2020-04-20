using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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
