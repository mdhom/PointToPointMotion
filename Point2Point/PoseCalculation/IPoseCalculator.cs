using System;
using System.Collections.Generic;
using System.Text;

namespace Point2Point.PoseCalculation
{
    public interface IPoseCalculator
    {
        bool CalculatesLocation { get; }
        bool CalculatesRotation { get; }

        IPose GetPose(TimeSpan time);
        IPose GetPose(TimeSpan time, out double acceleration, out double velocity, out double distance, out IP2PNode currentNode, out int currentEdgeIndex);
    }
}
