using Microsoft.Extensions.Logging;
using MoreLinq.Extensions;
using Point2Point.JointMotion;
using Point2Point.Mathematics.ExtendedP2P;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Point2Point.ClearanceHandling
{
    public class ClearanceHandlingMotionProfile : IClearanceHandlingMotionProfile
    {
        private readonly List<EdgeVelocityConstraint> _originalConstraints;
        private readonly MotionParameter _parameters;
        private readonly IEnumerable<IP2PDirectedEdge> _edges;
        private readonly double _vMax;
        private readonly ILogger _logger;

        private JointMotionProfile _jointMotionProfile;
        private double _jointMotionProfileCreatedTime;
        private double _jointMotionProfileCreatedDistance;
        public TimeSpan TotalDuration => TimeSpan.FromSeconds(_jointMotionProfile?.TotalDuration ?? 0);
        public double ClearedLength { get; private set; }
        public TimeSpan Created { get; }

        public ClearanceHandlingMotionProfile(TimeSpan created, IEnumerable<IP2PDirectedEdge> edges,
            MotionParameter parameters, double vMax, ILogger logger)
        {
            Created = created;
            _edges = edges;
            _parameters = parameters;
            _vMax = vMax;
            _logger = logger;
            _originalConstraints = GetConstraintsFrom(0);
        }

        private List<EdgeVelocityConstraint> GetConstraintsFrom(double distance)
        {
            var constraints = new List<EdgeVelocityConstraint>();
            var distanceSum = 0.0;
            var numSkipped = 0;
            const double epsilon = 0.000001;
            var edges = _edges.ToList();
            for (var i = 0; i < edges.Count; i++)
            {
                var edge = edges[i];
                var length = edge.Edge.Length();
                if (distanceSum + length > distance + epsilon || i == edges.Count - 1)
                {
                    var newConstraintLength = length - (distance - distanceSum);
                    constraints.Add(new EdgeVelocityConstraint(edge.Edge, 0.0, newConstraintLength,
                        Math.Min(edge.Edge.MaximumVelocity, _vMax)));
                    break;
                }
                else
                {
                    // edge is in front of the given distance -> skip this edge
                    distanceSum += length;
                    numSkipped++;
                }
            }

            foreach (var edge in _edges.Skip(numSkipped + 1))
            {
                constraints.Add(new EdgeVelocityConstraint(edge.Edge, constraints.Last().End,
                    Math.Min(edge.Edge.MaximumVelocity, _vMax)));
            }

            return constraints;
        }

        private bool IsUnclearedEdgeAvailable(int edgeId)
        {
            var distanceSum = 0.0;
            foreach (var edge in _edges)
            {
                var length = edge.Edge.Length();
                if (distanceSum + length > ClearedLength && edge.Edge.Id == edgeId)
                {
                    return true;
                }

                distanceSum += length;
            }

            _logger?.LogDebug(
                $"Clearance granted: Edge-{edgeId}, No uncleared edge available (distanceSum={distanceSum}, edges={string.Join(",", _edges.Select(e => e.Id))})!");

            return false;
        }

        public bool ClearanceGranted(int edgeId, TimeSpan time)
        {
            if (!IsUnclearedEdgeAvailable(edgeId))
            {
                return false;
            }

            var t = (time - Created).TotalSeconds;

            // determine status at time of clearance granting
            GetStatus(t, out var initialAcceleration, out var initialVelocity, out var initialDistance, out _, out _);

            // Get all constraints starting from initial distance. First constraint will be shortened
            var trimmedConstraintsList = GetConstraintsFrom(initialDistance);

            // Get constraints until 
            var clearedConstraintsList = trimmedConstraintsList.TakeUntil(c => c.CorrespondingEdge.Id == edgeId);

            if (clearedConstraintsList.Last().CorrespondingEdge.Id != edgeId)
            {
                _logger?.LogDebug(
                    $"Clearance granted WTF: Edge-{edgeId} => {string.Join(",", clearedConstraintsList.Select(c => c.CorrespondingEdge.Id))}");
            }

            _logger?.LogDebug(
                $"Clearance granted: Edge-{edgeId} => {string.Join(",", clearedConstraintsList.Select(c => c.CorrespondingEdge.Id))}");

            // create new motion profile
            _jointMotionProfile = new JointMotionProfile(_parameters, initialAcceleration, initialVelocity,
                clearedConstraintsList);
            _jointMotionProfileCreatedTime = t;
            _jointMotionProfileCreatedDistance = initialDistance;

            // increase cleared length
            ClearedLength = initialDistance + clearedConstraintsList.Last().End;

            return true;
        }

        public double GetS(double t)
        {
            return (_jointMotionProfile?.GetS(t - _jointMotionProfileCreatedTime) ?? 0.0) +
                   _jointMotionProfileCreatedDistance;
        }

        public double GetV(double t)
        {
            return _jointMotionProfile?.GetV(t - _jointMotionProfileCreatedTime) ?? 0.0;
        }

        public void GetStatus(double t, out double a, out double v, out double s)
            => GetStatus(t, out a, out v, out s, out _, out _);

        public void GetStatus(double t, out double a, out double v, out double s, out IP2PNode currentNode,
            out int currentEdgeIndex)
        {
            if (_jointMotionProfile != null)
            {
                _jointMotionProfile.GetStatus(t - _jointMotionProfileCreatedTime, out a, out v, out s);
                s += _jointMotionProfileCreatedDistance;
                TryGetCurrentEdgeIndex(s, out currentNode, out currentEdgeIndex);
            }
            else
            {
                a = 0;
                v = 0;
                s = 0;
                currentNode = null;
                currentEdgeIndex = 0;
            }
        }

        private bool TryGetCurrentEdgeIndex(double distance, out IP2PNode currentNode, out int edgeIndex)
        {
            edgeIndex = 0;
            var distanceSum = 0.0;
            var epsilon = 0.00001;
            foreach (var edge in _edges)
            {
                currentNode = edge.NodeFrom;
                var length = edge.Edge.Length();

                // Distance from MotionGenerator < Distance of edges in Map  --> still on the same edge
                if (distance < distanceSum + length - epsilon)
                {
                    return true;
                }

                //Target or point with zero velocity (i.e. Change in drive direction)
                if (Math.Abs(distanceSum + length - distance) < epsilon)
                {
                    currentNode = edge.NodeTo;
                    return true;
                }

                distanceSum += length;
                edgeIndex++;
            }

            currentNode = null;
            return false;
        }

        // public double GetAllVelocityPoints()
        // {
        //     _jointMotionProfile.VelocityProfilePoints
        // }

        public double GetTimeAtStartOf(IP2PEdge edge)
        {
            var constraint = _originalConstraints.First(c => c.CorrespondingEdge == edge);
            var timestamp = _jointMotionProfile.Timestamps.First(t => t.Distance == constraint.Start);
            return timestamp.Time;
        }

        public double GetTimeAtEndOf(IP2PEdge edge)
        {
            var constraint = _originalConstraints.First(c => c.CorrespondingEdge == edge);
            var timestamp =
                _jointMotionProfile.Timestamps.First(t => t.Distance == constraint.Start + constraint.Length);
            return timestamp.Time;
        }
    }
}