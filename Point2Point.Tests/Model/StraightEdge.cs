using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point.Tests.Model
{
    internal class StraightEdge : IP2PEdge
    {
        public int Id { get; set; }

        public IP2PNode StartNode { get; set; }
        public IP2PNode EndNode { get; set; }
        public double MaximumVelocity { get; set; }

        public Vector3 GetLocation(double distanceOnPath)
        {
            var straightVector = EndNode.Location - StartNode.Location;
            var length = straightVector.Length();
            var percentage = distanceOnPath / length;
            return StartNode.Location + (float)percentage * straightVector;
        }

        public Vector3 GetEndDirection() => Vector3.Normalize(EndNode.Location - StartNode.Location);
        public Vector3 GetStartDirection() => GetEndDirection();

        public double Length()
        {
            var straightVector = EndNode.Location - StartNode.Location;
            return straightVector.Length();
        }
    }
}