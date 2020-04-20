using System;
using System.Collections.Generic;
using System.ComponentModel;
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
                new VelocityPoint(0,0, null)
            };

            for (var i = 0; i < EffectiveConstraints.Count; i++)
            {
                var constraint = EffectiveConstraints[i];
                var nextConstraint = EffectiveConstraints.ElementAtOrDefault(i + 1);
                var availableDistance = unreachedConstraints.Sum(s => s.Length) + constraint.Length;
                var maxReachableVelocity = P2PCalculator.CalculateMaximumReachableVelocity(availableDistance, Parameters);
                var startDistance = velocityPoints.Max(v => v.Distance);
                var lastVelocity = velocityPoints.Last().Velocity;
                var velocityDifferenceBetweenConstraints = Math.Abs(constraint.MaximumVelocity - lastVelocity);
                if (maxReachableVelocity > velocityDifferenceBetweenConstraints)
                {
                    // maximum allowed velocity can be reached
                    var distanceForFullAcc = P2PCalculator.CalculateDistanceForAcceleration(velocityDifferenceBetweenConstraints, Parameters);
                    if (nextConstraint != null && nextConstraint.MaximumVelocity > constraint.MaximumVelocity)
                    {
                        // next constraint allows higher velocity 
                        // -> drive this constraint until end with constant velocity because it will be accelerated afterwards
                        velocityPoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                        velocityPoints.Add(new VelocityPoint(startDistance + availableDistance, constraint.MaximumVelocity, constraint));
                    }
                    else
                    {
                        // next constraint is below current constraint -> braking is necessary 
                        // OR
                        // no next constraint available -> brake to zero
                        var targetVelocity = nextConstraint?.MaximumVelocity ?? 0.0;
                        var velocityDifferenceToTargetVelocity = Math.Abs(constraint.MaximumVelocity - targetVelocity);
                        var distanceForBraking = P2PCalculator.CalculateDistanceForAcceleration(velocityDifferenceToTargetVelocity, Parameters);

                        if (distanceForFullAcc + distanceForBraking < availableDistance)
                        {
                            // constant velocity will be reached
                            velocityPoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                            velocityPoints.Add(new VelocityPoint(startDistance + (availableDistance - distanceForBraking), constraint.MaximumVelocity, constraint));
                            velocityPoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                        }
                        else if (distanceForFullAcc + distanceForBraking == availableDistance)
                        {
                            // constant velocity will not be reached but maximum velocity => exact peak
                            velocityPoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                            velocityPoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                        }
                        else
                        {
                            // we can not fully accelerate because we need to brake earlier than possible when only accelerating
                            // UNTIL I KNOW BETTER: iterative approach
                            var steppedPoints = IterativlyFindSteppedDownVelocity(lastVelocity, constraint, availableDistance, startDistance, targetVelocity);
                            if (steppedPoints != default)
                            {
                                velocityPoints.AddRange(steppedPoints);
                            }
                            else
                            {
                                StepBackByOneConstraint(constraint, velocityPoints, ref i);
                            }
                        }
                    }

                    unreachedConstraints.Clear();
                }
                else
                {
                    if (nextConstraint != null)
                    {
                        var diffToNextConstraint = Math.Abs(nextConstraint.MaximumVelocity - lastVelocity);
                        if (diffToNextConstraint > maxReachableVelocity)
                        {
                            if (nextConstraint.MaximumVelocity < lastVelocity)
                            {
                                StepBackByOneConstraint(constraint, velocityPoints, ref i);
                            }
                            else
                            {
                                MergeWithNextConstraint(constraint, ref i);
                            }
                        }
                        else
                        {
                            // after this constraint, next constraint is lower -> try find maximum reachable velocity
                            var steppedPoints = IterativlyFindSteppedDownVelocity(lastVelocity, constraint, availableDistance, startDistance, nextConstraint.MaximumVelocity);
                            if (steppedPoints != default)
                            {
                                velocityPoints.AddRange(steppedPoints);
                            }
                            else
                            {
                                StepBackByOneConstraint(constraint, velocityPoints, ref i);
                            }
                        }
                    }
                    else if (nextConstraint == null)
                    {
                        // no constraint any more -> brake to zero
                        var steppedPoints = IterativlyFindSteppedDownVelocity(lastVelocity, constraint, availableDistance, startDistance, 0.0);
                        if (steppedPoints != default)
                        {
                            velocityPoints.AddRange(steppedPoints);
                        }
                        else
                        {
                            StepBackByOneConstraint(constraint, velocityPoints, ref i);
                        }
                    }
                    else
                    {
                        MergeWithNextConstraint(constraint, ref i);
                    }
                }
            }

            return velocityPoints;
        }

        private void MergeWithNextConstraint(VelocityConstraint constraint, ref int index)
        {
            EffectiveConstraints.RemoveAt(index);
            EffectiveConstraints[index].Start  -= constraint.Length;
            EffectiveConstraints[index].Length += constraint.Length;

            index --;
        }

        private void StepBackByOneConstraint(VelocityConstraint constraint, List<VelocityPoint> velocityPoints, ref int index)
        {
            EffectiveConstraints.RemoveAt(index);
            if (index > 0)
            {
                EffectiveConstraints[index-1].Length += constraint.Length;
            }
            else
            {
                // ???
            }

            RemovePointsFromLastConstraint();

            index -= 2;

            void RemovePointsFromLastConstraint()
            {
                var correspondingConstraint = velocityPoints.Last().CorrespondingConstraint;
                for (int i = velocityPoints.Count - 1; i >= 0; i--)
                {
                    if (velocityPoints[i].CorrespondingConstraint == correspondingConstraint)
                    {
                        velocityPoints.RemoveAt(i);
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }

        private List<VelocityPoint> IterativlyFindSteppedDownVelocity(double lastVelocity, VelocityConstraint constraint, double availableDistance, double startDistance, double targetVelocity)
        {
            var velocityPoints = new List<VelocityPoint>();
            var targetAccVelocity = constraint.MaximumVelocity;
            var stepDownSize = 5.0;
            while (true)
            {
                targetAccVelocity -= stepDownSize;
                if (targetAccVelocity < lastVelocity)
                { 
                    // try with exactly zero (no acc at all) a last time
                    targetAccVelocity = lastVelocity;
                    if (!TryAddVelocityPoints())
                    {
                        Console.WriteLine($"No Step Down found");
                        return default;
                    }
                    break;
                }
                else if (TryAddVelocityPoints())
                {
                    break;
                }
            }

            return velocityPoints;

            bool TryAddVelocityPoints()
            {
                var velocityDifferenceToSDVelo = Math.Abs(targetAccVelocity - lastVelocity);
                var distanceForSDAcc = P2PCalculator.CalculateDistanceForAcceleration(velocityDifferenceToSDVelo, Parameters);
                var velocityDifferenceToTargetVelocityFromSD = Math.Abs(targetAccVelocity - targetVelocity);
                var distanceForBrakingFromSD = P2PCalculator.CalculateDistanceForAcceleration(velocityDifferenceToTargetVelocityFromSD, Parameters);
                if (distanceForSDAcc + distanceForBrakingFromSD < availableDistance)
                {
                    // constant velocity will be reached
                    velocityPoints.Add(new VelocityPoint(startDistance + distanceForSDAcc, targetAccVelocity, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + (availableDistance - distanceForBrakingFromSD), targetAccVelocity, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                    return true;
                }
                else if (distanceForSDAcc + distanceForBrakingFromSD == availableDistance)
                {
                    // constant velocity will not be reached but maximum velocity => exact peak
                    velocityPoints.Add(new VelocityPoint(startDistance + distanceForSDAcc, targetAccVelocity, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                    return true;
                }

                return false;
            }
        }
    }
}