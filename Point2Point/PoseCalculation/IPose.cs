using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point.PoseCalculation
{
    public interface IPose
    {
        Vector3 Location { get; }
        Vector3 Rotation { get; }
    }
}
