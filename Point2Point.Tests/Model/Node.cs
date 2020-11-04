using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point.Tests.Model
{
    internal class Node : IP2PNode
    {
        public int Id { get; set; }
        public Vector3 Location { get; set; }
    }
}
