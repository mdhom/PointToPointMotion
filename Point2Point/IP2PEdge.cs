using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point
{
    public interface IP2PEdge
    {
        int Id { get; }

        IP2PNode StartNode { get;  }
        IP2PNode EndNode { get;  }
        double MaximumVelocity { get; }

        double Length();

        Vector3 GetEndDirection();
        Vector3 GetStartDirection();
    }
}