using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point.Tests
{
    internal class DirectedEdge : IP2PDirectedEdge
    {
        public int Id => Edge.Id;

        public IP2PEdge Edge { get; }
        public DriveDirection Direction { get; }

        public Vector3 StartDirection => Direction == DriveDirection.Forward ? Edge.GetStartDirection() : Edge.GetEndDirection();
        public Vector3 EndDirection => Direction == DriveDirection.Forward ? Edge.GetEndDirection() : Edge.GetStartDirection();

        public DirectedEdge(IP2PEdge edge, DriveDirection direction = DriveDirection.Forward)
        {
            Edge = edge;
            Direction = direction;
        }

        public DirectedEdge(IP2PEdge edge, int nodeFromId)
        {
            Edge = edge;
            Direction = nodeFromId == Edge.StartNode.Id ? DriveDirection.Forward : DriveDirection.Backward;
        }

        public Vector3 GetLocation(float distanceOnPath)
        {
            throw new NotImplementedException();
        }

        public Vector3 GetRotation(float distanceOnPath)
        {
            throw new NotImplementedException();
        }
    }
}