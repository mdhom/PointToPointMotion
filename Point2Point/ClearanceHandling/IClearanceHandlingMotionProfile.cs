using Point2Point.JointMotion;
using System;
using System.Collections.Generic;
using System.Text;

namespace Point2Point.ClearanceHandling
{
    public interface IClearanceHandlingMotionProfile : IMotionProfile
    {
        double ClearedLength { get; }

        bool ClearanceGranted(int edgeId, TimeSpan time);

        void GetStatus(double t, out double a, out double v, out double s, out IP2PNode currentNode, out int currentEdgeIndex);
    }
}
