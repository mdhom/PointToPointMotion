﻿using System;
using System.Collections.Generic;
using System.Linq;
using Shuttles.Base.Devices.Shuttles.Motion.Ramp;

namespace Point2Point.JointMotion
{
    public class JointMotionProfile
    {
        public RampMotionParameter Parameters { get; }

        public List<VelocityConstraint> EffectiveConstraints { get; private set; }
        public List<VelocityPoint> VelocityProfilePoints { get; private set; }
        public List<RampCalculationResult> RampResults { get; private set; }
        public List<double> TimesAtVelocityPoints { get; private set; }
        public double TotalDuration { get; private set; }

        public JointMotionProfile(RampMotionParameter parameters, ConstraintsCollection constraints)
        {
            Parameters = parameters;

            CalculateProfile(constraints);
        }

        public JointMotionProfile(RampMotionParameter parameters, IEnumerable<VelocityConstraint> constraints)
            : this(parameters, new ConstraintsCollection(constraints))
        {
        }

        public JointMotionProfile(RampMotionParameter parameters, VelocityConstraint constraint, params VelocityConstraint[] constraints)
            : this(parameters, new List<VelocityConstraint>() { constraint }.Concat(constraints))
        {
        }

        public double GetV(double t)
        {
            GetStatus(t, out var v, out _);
            return v;
        }

        public double GetS(double t)
        {
            GetStatus(t, out _, out var s);
            return s;
        }

        public void GetStatus(double t, out double v, out double s)
        {
            t = Math.Min(t, TotalDuration);

            var pointToIndex = TimesAtVelocityPoints.FindIndex(tAtPoint => tAtPoint > t) + 1;
            if (pointToIndex == 0)
            {
                pointToIndex = VelocityProfilePoints.Count - 1;
            }
            var pointFromIndex = pointToIndex - 1;

            var pointFrom = VelocityProfilePoints[pointFromIndex];
            var pointTo = VelocityProfilePoints[pointToIndex];

            var tFrom = TimesAtVelocityPoints.ElementAtOrDefault(pointFromIndex - 1);
            var tInRamp = t - tFrom;

            if (pointFrom.Velocity == pointTo.Velocity)
            {
                v = pointFrom.Velocity;
                s = tInRamp * v;
            }
            else
            {
                var rampresult = RampResults.ElementAtOrDefault(pointFromIndex);
                RampCalculator.CalculateStatus(rampresult, tInRamp, out _, out _, out v, out s);
            }

            s += pointFrom.Distance;
        }

        private void CalculateProfile(ConstraintsCollection constraints)
        {
            var effectiveConstraintsOriginal = constraints.GetEffectiveConstraints();
            EffectiveConstraints = effectiveConstraintsOriginal.Select(c => c.Copy()).ToList();

            var profilePoints = new List<VelocityPoint>()
            {
                new VelocityPoint(0,0, null)
            };

            for (var i = 0; i < EffectiveConstraints.Count; i++)
            {
                var constraint = EffectiveConstraints[i];
                var nextConstraint = EffectiveConstraints.ElementAtOrDefault(i + 1);

                var availableDistance = constraint.Length;
                var startDistance = profilePoints.Max(v => v.Distance);
                var startVelocity = profilePoints.Last().Velocity;
                var targetVelocity = constraint.MaximumVelocity;

                if (AreSameVelocities(startVelocity, targetVelocity) || RampCalculator.IsReachable(startVelocity, targetVelocity, availableDistance, Parameters))
                {
                    // maximum allowed velocity can be reached
                    var distanceForFullAcc = RampCalculator.CalculateDistanceNeeded(startVelocity, constraint.MaximumVelocity, Parameters);
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
                        targetVelocity = nextConstraint?.MaximumVelocity ?? 0.0;
                        var distanceForBraking = RampCalculator.CalculateDistanceNeeded(constraint.MaximumVelocity, targetVelocity, Parameters);

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
                            IterativlyFindSteppedDownVelocity(startVelocity, constraint, availableDistance, startDistance, targetVelocity, profilePoints, ref i);
                        }
                    }
                }
                else if (nextConstraint != null)
                {
                    // maximum allowed velocity can't be reached and there is at least one constraint to follow
                    if (!RampCalculator.IsReachable(startVelocity, nextConstraint.MaximumVelocity, availableDistance + nextConstraint.Length, Parameters))
                    {
                        // next constraint can not be reached from current constraint
                        if (nextConstraint.MaximumVelocity < startVelocity)
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
                        IterativlyFindSteppedDownVelocity(startVelocity, constraint, availableDistance, startDistance, nextConstraint.MaximumVelocity, profilePoints, ref i);
                    }
                }
                else
                {
                    // no constraint any more -> brake to zero
                    IterativlyFindSteppedDownVelocity(startVelocity, constraint, availableDistance, startDistance, 0.0, profilePoints, ref i);
                }
            }

            // remove ProfilePoints with same distance and velocity
            profilePoints = profilePoints.DistinctBy(pp => new { pp.Distance, pp.Velocity }).ToList();

            // Calculate ramp results and times
            var rampResults = new List<RampCalculationResult>();
            var times = new List<double>();
            var timeSum = 0.0;
            for (var i = 0; i < profilePoints.Count - 1; i++)
            {
                var pFrom = profilePoints[i];
                var pTo = profilePoints[i + 1];

                var ramp = RampCalculator.Calculate(pFrom.Velocity, pTo.Velocity, Parameters);
                rampResults.Add(ramp);

                if (!ramp.Flat && Math.Abs(ramp.TotalDistance - (pTo.Distance - pFrom.Distance)) > 1e-8)
                {
                    // Ouch!
                }

                var duration = ramp.Flat ? (pTo.Distance - pFrom.Distance) / pTo.Velocity : ramp.TotalDuration;
                if (double.IsNaN(duration) || double.IsInfinity(duration))
                {
                    throw new JointMotionCalculationException($"Invalid duration ({duration}) on point {i} at {pFrom.Distance}");
                }
                timeSum += duration;
                times.Add(timeSum);
            }

            VelocityProfilePoints = profilePoints;
            RampResults = rampResults;
            TimesAtVelocityPoints = times;
            TotalDuration = timeSum;

            if (double.IsNaN(TotalDuration) || double.IsInfinity(TotalDuration))
            {
                throw new JointMotionCalculationException($"Invalid TotalDuration ({TotalDuration})");
            }
        }

        /// <summary>
        /// Removes the given constraint and adds its length to the next constraint.
        /// </summary>
        private void MergeWithNextConstraint(VelocityConstraint constraint, ref int index)
        {
            EffectiveConstraints.RemoveAt(index);
            EffectiveConstraints[index].Start -= constraint.Length;
            EffectiveConstraints[index].Length += constraint.Length;

            index--;
        }

        /// <summary>
        /// Removes the given constraint and adds its length to the previous constraint 
        /// => the previous constraint is longer now. Enables earlier braking.
        /// </summary>
        private void MergeWithPreviousConstraint(VelocityConstraint constraint, List<VelocityPoint> velocityPoints, ref int index)
        {
            EffectiveConstraints.RemoveAt(index);
            if (index > 0)
            {
                EffectiveConstraints[index - 1].Length += constraint.Length;
                EffectiveConstraints[index - 1].MaximumVelocity = Math.Min(constraint.MaximumVelocity, EffectiveConstraints[index - 1].MaximumVelocity);
            }
            else
            {
                // Don't know if this could happen ???
            }

            var correspondingConstraint = velocityPoints.Last().CorrespondingConstraint;
            velocityPoints.RemoveAll(vp => vp.CorrespondingConstraint == correspondingConstraint);

            index -= 2;
        }

        private void IterativlyFindSteppedDownVelocity(double lastVelocity, VelocityConstraint constraint, double availableDistance, double startDistance, double targetVelocity, List<VelocityPoint> velocityPoints, ref int index)
        {
            var targetAccVelocity = constraint.MaximumVelocity;
            var stepDownSize = 5.0;
            while (true)
            {
                targetAccVelocity -= stepDownSize;
                if (targetAccVelocity <= lastVelocity)
                {
                    // try with lastVelocity last time because we may have overstepped
                    // that critial point
                    if (lastVelocity == 0 || !TryAddVelocityPoints(lastVelocity))
                    {
                        // did not find a possible targetVelocity ->
                        // iterave search failed -> step back
                        if (targetVelocity > lastVelocity)
                        {
                            MergeWithNextConstraint(constraint, ref index);
                        }
                        else
                        {
                            MergeWithPreviousConstraint(constraint, velocityPoints, ref index);
                        }
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
                var distanceForSDAcc = RampCalculator.CalculateDistanceNeeded(lastVelocity, targetVel, Parameters);
                var distanceForBrakingFromSD = RampCalculator.CalculateDistanceNeeded(targetVel, targetVelocity, Parameters);
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

        private static bool AreSameVelocities(double velo1, double velo2)
            => Math.Abs(velo1 - velo2) < 1e-8;
    }
}