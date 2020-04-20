using System;
using System.Collections.Generic;
using System.Linq;

namespace Point2Point.JointMotion
{
    public class JointMotionProfile
    {
        private readonly List<VelocityConstraint> _effectiveConstraints;
        private readonly P2PParameters _parameters;

        private JointMotionProfile(ConstraintsCollection constraints, P2PParameters parameters)
        {
            _effectiveConstraints = constraints.GetEffectiveConstraints();
            _parameters = parameters;
        }

        public static List<VelocityPoint> CalculateProfile(ConstraintsCollection constraints, P2PParameters parameters)
        {
            var profile = new JointMotionProfile(constraints, parameters);
            return profile.CalculateProfile();
        }

        private List<VelocityPoint> CalculateProfile()
        {
            var profilePoints = new List<VelocityPoint>()
            {
                new VelocityPoint(0,0, null)
            };

            for (var i = 0; i < _effectiveConstraints.Count; i++)
            {
                var constraint = _effectiveConstraints[i];
                var nextConstraint = _effectiveConstraints.ElementAtOrDefault(i + 1);
                var availableDistance = constraint.Length;
                var maxReachableVelocity = P2PCalculator.CalculateMaximumReachableVelocity(availableDistance, _parameters);
                var startDistance = profilePoints.Max(v => v.Distance);
                var lastVelocity = profilePoints.Last().Velocity;
                var velocityDifferenceBetweenConstraints = Math.Abs(constraint.MaximumVelocity - lastVelocity);
                if (maxReachableVelocity > velocityDifferenceBetweenConstraints)
                {
                    // maximum allowed velocity can be reached
                    var distanceForFullAcc = P2PCalculator.CalculateDistanceForAcceleration(velocityDifferenceBetweenConstraints, _parameters);
                    if (nextConstraint != null && nextConstraint.MaximumVelocity > constraint.MaximumVelocity)
                    {
                        // next constraint allows higher velocity 
                        // -> drive this constraint until end with constant velocity because it will be accelerated afterwards
                        profilePoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                        profilePoints.Add(new VelocityPoint(startDistance + availableDistance, constraint.MaximumVelocity, constraint));
                    }
                    else
                    {
                        // next constraint is below current constraint -> braking is necessary 
                        // OR
                        // no next constraint available -> brake to zero
                        var targetVelocity = nextConstraint?.MaximumVelocity ?? 0.0;
                        var velocityDifferenceToTargetVelocity = Math.Abs(constraint.MaximumVelocity - targetVelocity);
                        var distanceForBraking = P2PCalculator.CalculateDistanceForAcceleration(velocityDifferenceToTargetVelocity, _parameters);

                        if (distanceForFullAcc + distanceForBraking < availableDistance)
                        {
                            // constant velocity will be reached
                            profilePoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                            profilePoints.Add(new VelocityPoint(startDistance + (availableDistance - distanceForBraking), constraint.MaximumVelocity, constraint));
                            profilePoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                        }
                        else if (distanceForFullAcc + distanceForBraking == availableDistance)
                        {
                            // constant velocity will not be reached but maximum velocity => exact peak
                            profilePoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                            profilePoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                        }
                        else
                        {
                            // we can not fully accelerate because we need to brake earlier than possible when only accelerating
                            // UNTIL I KNOW BETTER: iterative approach
                            IterativlyFindSteppedDownVelocity(lastVelocity, constraint, availableDistance, startDistance, targetVelocity, profilePoints, ref i);
                        }
                    }
                }
                else if (nextConstraint != null)
                {
                    // there is at least one constraint to follow
                    var diffToNextConstraint = Math.Abs(nextConstraint.MaximumVelocity - lastVelocity);
                    if (diffToNextConstraint > maxReachableVelocity)
                    {
                        // next constraint can not be reached from current constraint
                        if (nextConstraint.MaximumVelocity < lastVelocity)
                        {
                            // next constraint is a step downwards, so there could be a
                            // critical intersection with the constraints
                            // -> step back and try earlier braking
                            MergeWithPreviousConstraint(constraint, profilePoints, ref i);
                        }
                        else
                        {
                            // next constraint is a step upwards, so no 
                            // critical intersection with constraints can happen
                            // -> merge this constraint with next constraint for longer acceleration distance
                            MergeWithNextConstraint(constraint, ref i);
                        }
                    }
                    else
                    {
                        // after this constraint, next constraint is lower -> try find maximum reachable velocity
                        IterativlyFindSteppedDownVelocity(lastVelocity, constraint, availableDistance, startDistance, nextConstraint.MaximumVelocity, profilePoints, ref i);
                    }
                }
                else
                {
                    // no constraint any more -> brake to zero
                    IterativlyFindSteppedDownVelocity(lastVelocity, constraint, availableDistance, startDistance, 0.0, profilePoints, ref i);
                }
            }

            return profilePoints;
        }

        /// <summary>
        /// Removes the given constraint and adds its length to the next constraint.
        /// </summary>
        private void MergeWithNextConstraint(VelocityConstraint constraint, ref int index)
        {
            _effectiveConstraints.RemoveAt(index);
            _effectiveConstraints[index].Start -= constraint.Length;
            _effectiveConstraints[index].Length += constraint.Length;

            index--;
        }

        /// <summary>
        /// Removes the given constraint and adds its length to the previous constraint 
        /// => the previous constraint is longer now. Enables earlier braking.
        /// </summary>
        private void MergeWithPreviousConstraint(VelocityConstraint constraint, List<VelocityPoint> velocityPoints, ref int index)
        {
            _effectiveConstraints.RemoveAt(index);
            if (index > 0)
            {
                _effectiveConstraints[index - 1].Length += constraint.Length;
            }
            else
            {
                // Don't know if this could happen ???
            }

            RemoveVelocityPointsFromLastConstraint();

            index -= 2;

            void RemoveVelocityPointsFromLastConstraint()
            {
                var correspondingConstraint = velocityPoints.Last().CorrespondingConstraint;
                velocityPoints.RemoveAll(vp => vp.CorrespondingConstraint == correspondingConstraint);
            }
        }

        private void IterativlyFindSteppedDownVelocity(double lastVelocity, VelocityConstraint constraint, double availableDistance, double startDistance, double targetVelocity, List<VelocityPoint> velocityPoints, ref int index)
        {
            var targetAccVelocity = constraint.MaximumVelocity;
            var stepDownSize = 5.0;
            while (true)
            {
                targetAccVelocity -= stepDownSize;
                if (targetAccVelocity < lastVelocity)
                {
                    // try with lastVelocity last time because we may have overstepped
                    // that critial point
                    if (!TryAddVelocityPoints(lastVelocity))
                    {
                        // did not find a possible targetVelocity ->
                        // iterave search failed -> step back
                        MergeWithPreviousConstraint(constraint, velocityPoints, ref index);
                    }
                    return;
                }
                else if (TryAddVelocityPoints(targetAccVelocity))
                {
                    // successfully found a new targetVelocity which allows
                    // for accelerating and braking
                    break;
                }
            }

            bool TryAddVelocityPoints(double targetVel)
            {
                var velocityDifferenceToSDVelo = Math.Abs(targetVel - lastVelocity);
                var distanceForSDAcc = P2PCalculator.CalculateDistanceForAcceleration(velocityDifferenceToSDVelo, _parameters);
                var velocityDifferenceToTargetVelocityFromSD = Math.Abs(targetVel - targetVelocity);
                var distanceForBrakingFromSD = P2PCalculator.CalculateDistanceForAcceleration(velocityDifferenceToTargetVelocityFromSD, _parameters);
                if (distanceForSDAcc + distanceForBrakingFromSD < availableDistance)
                {
                    // constant velocity will be reached
                    velocityPoints.Add(new VelocityPoint(startDistance + distanceForSDAcc, targetVel, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + (availableDistance - distanceForBrakingFromSD), targetVel, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                    return true;
                }
                else if (distanceForSDAcc + distanceForBrakingFromSD == availableDistance)
                {
                    // constant velocity will not be reached but maximum velocity => exact peak
                    velocityPoints.Add(new VelocityPoint(startDistance + distanceForSDAcc, targetVel, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                    return true;
                }

                return false;
            }
        }
    }
}