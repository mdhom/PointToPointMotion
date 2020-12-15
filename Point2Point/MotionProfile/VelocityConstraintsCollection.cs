﻿using System.Collections.Generic;
using System.Linq;

namespace Point2Point.JointMotion
{
    public class VelocityConstraintsCollection : List<VelocityConstraint>
    {
        #region Constructors

        public VelocityConstraintsCollection()
        {
        }

        public VelocityConstraintsCollection(IEnumerable<VelocityConstraint> constraints)
            : base(constraints)
        {
        }

        public VelocityConstraintsCollection(VelocityConstraint constraint, params VelocityConstraint[] constraints)
            : base(new List<VelocityConstraint>() { constraint }.Concat(constraints))
        {
        }

        #endregion

        /// <summary>
        /// Calculates the effective constraints of the ConstraintsCollection. Constraints or part of
        /// constraints which are above other constraints (and therefore will never be reached by a
        /// velocity profile) are removed and merged into a closed profile of constraints.
        /// </summary>
        /// <returns>A new ConstraintCollection with no overlapping constraints</returns>
        public VelocityConstraintsCollection GetEffectiveConstraints()
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

            var effectiveConstraints = new VelocityConstraintsCollection();
            for (var i = 0; i < minPoints.Count; i++)
            {
                var from = minPoints[i];
                var endIndex = minPoints.FindIndex(i + 1, sp => sp.Velocity != from.Velocity);
                if (endIndex == -1)
                {
                    // constraint with other velocity further found
                    effectiveConstraints.Add(new VelocityConstraint(from.Distance, this.Max(s => s.End) - from.Distance, from.Velocity, "GetEffectiveConstraints"));
                    return effectiveConstraints;
                }
                else
                {
                    var to = minPoints[endIndex];
                    effectiveConstraints.Add(new VelocityConstraint(from.Distance, to.Distance - from.Distance, from.Velocity, "GetEffectiveConstraints"));
                }

                i = endIndex - 1;
            }

            return effectiveConstraints;
        }

        /// <summary>
        /// Checks wether there is any constraint at the given distance [mm] which is below the given velocity [mm/s].
        /// </summary>
        /// <param name="distance">Distance [mm] at which the constraints should be checked</param>
        /// <param name="velocity">Velocity [mm/s] which should be checked</param>
        /// <returns>True if any constraint is below the given velocity at the given distance, otherwise false</returns>
        public bool ExeedsAnyConstraint(double distance, double velocity)
            => this.Any(c => c.Contains(distance) && velocity > c);

        struct ConstraintPoint
        {
            public double Distance { get; }
            public double Velocity { get; }

            public ConstraintPoint(double distance, double velocity)
            {
                Distance = distance;
                Velocity = velocity;
            }
        }
    }
}