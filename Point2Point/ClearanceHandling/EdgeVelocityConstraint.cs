using Point2Point.JointMotion;
using System;
using System.Collections.Generic;
using System.Text;

namespace Point2Point.ClearanceHandling
{
    public class EdgeVelocityConstraint : VelocityConstraint
    {
        public IP2PEdge CorrespondingEdge { get; set; }

        public EdgeVelocityConstraint(IP2PEdge correspondingEdge, double start, double maximumVelocity)
            : base(start, correspondingEdge.Length(), maximumVelocity,"EdgeVelocityConstraint")
        {
            CorrespondingEdge = correspondingEdge;
        }

        public EdgeVelocityConstraint(IP2PEdge correspondingEdge, double start, double manualLength, double maximumVelocity)
            : base(start, manualLength, maximumVelocity, "EdgeVelocityConstraint")
        {
            CorrespondingEdge = correspondingEdge;
        }
    }
}
