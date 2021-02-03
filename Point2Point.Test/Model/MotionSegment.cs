using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Point2Point.Tests.Model
{
    internal class MotionSegment
    {
        public List<DirectedEdge> DirectedEdges { get; set; } = new List<DirectedEdge>();
        public IEnumerable<IP2PEdge> Edges => DirectedEdges.Select(e => e.Edge);
    }
}
