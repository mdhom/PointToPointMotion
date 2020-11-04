using MoreLinq;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;

namespace Point2Point.Tests.Model
{
    internal class Map
    {
        public List<Node> Nodes { get; set; }
        public List<StraightEdge> Edges { get; set; }

        public Map()
        {
            Nodes = new List<Node>();
            Edges = new List<StraightEdge>();
        }

        public Node AddNodeAt(Vector3 vector)
        {
            var node = new Node()
            {
                Id = Nodes.Any() ? Nodes.Max(n => n.Id) + 1 : 1,
                Location = vector
            };
            Nodes.Add(node);
            return node;
        }

        public Node AddNodeAt(float x, float y, float z)
            => AddNodeAt(new Vector3(x, y, z));

        public Node AddOrGetNodeAt(Vector3 vector)
        {
            var closeNodes = GetNodesAt(vector.X, vector.Y, vector.Z);
            if (closeNodes.Any())
            {
                return closeNodes.First();
            }

            return AddNodeAt(vector);
        }

        public Node AddOrGetNodeAt(float x, float y, float z)
            => AddOrGetNodeAt(new Vector3(x, y, z));

        public IEnumerable<Node> GetNodesAt(float x, float y, float z)
            => Nodes.Where(n => Math.Abs(n.Location.X - x) < 0.1 && Math.Abs(n.Location.Y - y) < 0.1 && Math.Abs(n.Location.Z - z) < 0.1);

        public StraightEdge Connect(Node startNode, Node endNode)
        {
            var edge = new StraightEdge()
            {
                Id = Edges.Any() ? Edges.Max(n => n.Id) + 1 : 1,
                StartNode = startNode,
                EndNode = endNode
            };
            Edges.Add(edge);
            return edge;
        }

        public StraightEdge Connect(int startNodeId, int endNodeId)
            => Connect(GetNode(startNodeId), GetNode(endNodeId));

        public StraightEdge Connect(Node startNode, int endNodeId)
            => Connect(startNode, GetNode(endNodeId));

        public StraightEdge Connect(int startNodeId, Node endNode)
            => Connect(GetNode(startNodeId), endNode);

        public Node GetNode(int nodeId)
            => Nodes.FirstOrDefault(n => n.Id == nodeId);

        public StraightEdge GetEdge(int startNodeId, int endNodeId)
            => Edges.FirstOrDefault(e => (e.StartNode.Id == startNodeId && e.EndNode.Id == endNodeId) || (e.EndNode.Id == startNodeId && e.StartNode.Id == endNodeId));

        public StraightEdge GetEdge(Node startNode, Node endNode)
            => GetEdge(startNode.Id, endNode.Id);

        public StraightEdge GetEdge(int edgeId)
            => Edges.FirstOrDefault(e => e.Id == edgeId);

        public List<MotionSegment> GetMotionSegments(IEnumerable<int> pointIds)
        {
            var segments = new List<MotionSegment>();
            DirectedEdge previousEdge = null;
            for (int i = 0; i < pointIds.Count() - 1; i++)
            {
                var nodeFromId = pointIds.ElementAt(i);
                var nodeFrom = GetNode(nodeFromId);
                var nodeTo = GetNode(pointIds.ElementAt(i + 1));
                var edge = GetEdge(nodeFrom, nodeTo);
                var directedEdge = new DirectedEdge(edge, nodeFromId);

                if (previousEdge == null || !previousEdge.EndDirection.IsCollinearWith(directedEdge.StartDirection))
                {
                    segments.Add(new MotionSegment());
                }

                segments.Last().DirectedEdges.Add(directedEdge);
                previousEdge = directedEdge;
            }
            return segments;
        }

        public Node MergeNodes(Node node0, Node node1)
        {
            Nodes.Remove(node1);
            Edges.Where(e => e.StartNode == node1).ForEach(e => e.StartNode = node0);
            Edges.Where(e => e.EndNode == node1).ForEach(e => e.EndNode = node0);
            return node0;
        }
    }
}
