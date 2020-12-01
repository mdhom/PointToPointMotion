using Point2Point.PoseCalculation;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Point2Point
{
    public static class IEnumerableEdgeExtensions
    {
        public static IEnumerable<IP2PEdge> TakeDrivenAndCurrentEdge(this IEnumerable<IP2PEdge> edges, float distance)
        {
            var sumDistances = 0.0;
            return edges.TakeWhile(e =>
            {
                if (sumDistances > distance)
                {
                    return false;
                }
                else
                {
                    var length = e.Length();
                    sumDistances += length;
                    return true;
                }
            });
        }

        public static (IP2PDirectedEdge edge, double distanceOnEdge) GetCurrentEdge(this IEnumerable<IP2PDirectedEdge> edges, float distance)
        {
            var sumDistances = 0d;
            foreach (var edge in edges)
            {
                var length = edge.Length();
                if (sumDistances + length > distance)
                {
                    return (edge, distance - sumDistances);
                }

                sumDistances += length;
            }

            return (edges.Last(), edges.Last().Length());
        }

        public static IPose GetCurrentPose(this IEnumerable<IP2PDirectedEdge> edges, float distance)
        {
            (var edge, var distanceOnEdge) = edges.GetCurrentEdge(distance);
            var rotation = edge.GetRotation(distanceOnEdge);
            return new Pose(edge.GetLocation(distanceOnEdge), rotation); 
        }
    }
}