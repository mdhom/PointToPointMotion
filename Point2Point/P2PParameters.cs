using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Point2Point
{
    public class P2PParameters
    {
        public double JerkMax { get; }
        public double AccelerationMax { get; }
        public double VelocityMax { get; }

        public P2PParameters(double jerkMax, double accelerationMax, double velocityMax)
        {
            JerkMax = jerkMax;
            AccelerationMax = accelerationMax;
            VelocityMax = velocityMax;
        }
    }
}
