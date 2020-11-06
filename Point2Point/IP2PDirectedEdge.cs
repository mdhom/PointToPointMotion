using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point
{
    public interface IP2PDirectedEdge
    {
        int Id { get; }
        IP2PEdge Edge { get; }
        DriveDirection Direction { get; }

        IP2PNode NodeFrom => Direction == DriveDirection.Forward ? Edge.StartNode : Edge.EndNode;
        IP2PNode NodeTo => Direction == DriveDirection.Forward ? Edge.EndNode : Edge.StartNode;

        Vector3 StartDirection => Direction == DriveDirection.Forward ? Edge.GetStartDirection() : Edge.GetEndDirection();
        Vector3 EndDirection => Direction == DriveDirection.Forward ? Edge.GetEndDirection() : Edge.GetStartDirection();

        float Length() => Edge.Length();

        Vector3 GetLocation(float distanceOnPath);
        Vector3 GetRotation(float distanceOnPath);
    }
}
