using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point
{
    public interface IP2PNode
    {
        int Id { get; set; }

        Vector3 Location { get; set; }
    }
}
