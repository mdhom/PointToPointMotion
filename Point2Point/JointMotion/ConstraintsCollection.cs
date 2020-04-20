using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;

namespace Point2Point.JointMotion
{
    public class ConstraintsCollection : List<VelocityConstraint>
    {
        public ConstraintsCollection()
        {
        }

        public ConstraintsCollection(IEnumerable<VelocityConstraint> constraints)
            : base(constraints)
        {
        }

        public ConstraintsCollection(VelocityConstraint constraint, params VelocityConstraint[] constraints)
            : base (new List<VelocityConstraint>() { constraint }.Concat(constraints))
        {
        }

        public void Append(VelocityConstraint segment, params VelocityConstraint[] segments)
            => Append(new List<VelocityConstraint>() { segment }.Concat(segments));

        public void Append(IEnumerable<VelocityConstraint> segments)
        {
            foreach (var segment in segments)
            {
                segment.Start = this.Any() ? this.Max(s => s.End) : 0;
                Add(segment);
            }
        }

        public void Insert(VelocityConstraint segment)
        {
            Add(segment);
        }

        public List<VelocityConstraint> GetEffectiveConstraints()
        {
            var distanceValues = this
                .SelectMany(s => new[] { s.Start, s.End })
                .Distinct()
                .OrderBy(d => d)
                .ToList();

            var minPoints = new List<ConstraintPoint>();
            foreach (var distance in distanceValues)
            {
                var containingSegments = this.Where(s => s.Contains(distance));
                if (containingSegments.Any())
                {
                    var minAllowedVelocity = containingSegments.Min(s => s.MaximumVelocity);
                    minPoints.Add(new ConstraintPoint(distance, minAllowedVelocity));
                }
            }

            var effectiveConstraints = new List<VelocityConstraint>();
            for (var i = 0; i < minPoints.Count; i++)
            {
                var from = minPoints[i];
                var endIndex = minPoints.FindIndex(i + 1, sp => sp.Velocity != from.Velocity);
                if (endIndex == -1)
                {
                    // constraint with other velocity further found
                    effectiveConstraints.Add(new VelocityConstraint(from.Distance, this.Max(s => s.End) - from.Distance, from.Velocity));
                    return effectiveConstraints;
                }
                else
                {
                    var to = minPoints[endIndex];
                    effectiveConstraints.Add(new VelocityConstraint(from.Distance, to.Distance - from.Distance, from.Velocity));
                }

                i = endIndex - 1;
            }

            return effectiveConstraints;
        }

        private struct ConstraintPoint
        {
            public double Distance { get; set; }
            public double Velocity { get; set; }

            public ConstraintPoint(double distance, double velocity)
            {
                Distance = distance;
                Velocity = velocity;
            }
        }
    }
}
