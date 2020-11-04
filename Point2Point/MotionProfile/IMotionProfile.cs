using System;
using System.Collections.Generic;
using System.Text;

namespace Point2Point.JointMotion
{
    public interface IMotionProfile
    {
        double GetV(double t);
        double GetS(double t);
        void GetStatus(double t, out double a, out double v, out double s);
    }
}
