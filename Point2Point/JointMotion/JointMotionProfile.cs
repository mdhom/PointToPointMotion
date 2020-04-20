using System;
using System.Collections.Generic;
using System.Linq;

namespace Point2Point.JointMotion
{
    public partial class JointMotionProfile
    {
        public List<VelocityConstraint> EffectiveConstraints { get; }
        public P2PParameters Parameters { get; }

        public JointMotionProfile(ConstraintsCollection constraints, P2PParameters parameters)
        {
            EffectiveConstraints = constraints.GetEffectiveConstraints();
            Parameters = parameters;
        }

        public List<VelocityPoint> CalculateProfile()
        {
            var unreachedConstraints = new List<VelocityConstraint>();
            var velocityPoints = new List<VelocityPoint>()
            {
                new VelocityPoint(0,0)
            };

            for (var i = 0; i < EffectiveConstraints.Count; i++)
            {
                var constraint = EffectiveConstraints[i];
                var availableDistance = unreachedConstraints.Sum(s => s.Length) + constraint.Length;
                var maxReachableVelocity = P2PCalculator.CalculateMaximumReachableVelocity(availableDistance, Parameters);
                var velocityDifferenceBetweenConstraints = Math.Abs(constraint.MaximumVelocity - velocityPoints.Last().Velocity);
                if (maxReachableVelocity > velocityDifferenceBetweenConstraints)
                {
                    // maximum allowed velocity can be reached
                    var distanceForFullAcc = P2PCalculator.GetDistanceForFullAcceleration(velocityDifferenceBetweenConstraints, Parameters);
                    var startDistance = velocityPoints.Max(v => v.Distance);
                    velocityPoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity));
                    velocityPoints.Add(new VelocityPoint(startDistance + constraint.Length, constraint.MaximumVelocity));
                    unreachedConstraints.Clear();
                }
                else
                {
                    unreachedConstraints.Add(constraint);
                }
            }

            return velocityPoints;
        }
    }
}
