using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
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

        public List<List<VelocityConstraint>> EffectiveConstraintsHistory { get; private set; }

        public JointMotionProfile(RampMotionParameter parameters, ConstraintsCollection constraints)
        {
            Parameters = parameters;

            EffectiveConstraints = constraints.GetEffectiveConstraints();

            EffectiveConstraintsHistory = new List<List<VelocityConstraint>>();
            while (true)
            {
                EffectiveConstraintsHistory.Add(EffectiveConstraints.Select(ec => ec.Copy()).ToList());
                if (CalculateProfile())
                {
                    return;
                }
            }
        }

        public JointMotionProfile(RampMotionParameter parameters, ConstraintsCollection constraints, SemaphoreSlim stepsSemaphore)
        {
            Parameters = parameters;

            EffectiveConstraints = constraints.GetEffectiveConstraints();

            EffectiveConstraintsHistory = new List<List<VelocityConstraint>>();

            Task.Run(async () =>
            {
                while (true)
                {
                    EffectiveConstraintsHistory.Add(EffectiveConstraints.Select(ec => ec.Copy()).ToList());
                    if (CalculateProfile())
                    {
                        return;
                    }
                    else
                    {
                        await stepsSemaphore.WaitAsync();
                    }
                }
            });
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

        private bool CalculateProfile()
        {
            var profilePoints = new List<VelocityPoint>()
            {
                new VelocityPoint(0,0, null)
            };

            for (var i = 0; i < EffectiveConstraints.Count; i++)
            {
                var constraint = EffectiveConstraints[i];
                var nextConstraint = EffectiveConstraints.ElementAtOrDefault(i + 1);

                var v0 = profilePoints.Last().Velocity;
                var v1 = constraint.MaximumVelocity;
                var v2 = nextConstraint?.MaximumVelocity ?? 0.0;

                var availableDistance = constraint.Length;
                var startDistance = profilePoints.Max(v => v.Distance);

                var situation = GetSituation(v0, constraint, nextConstraint);
                if (situation == 7)
                {
                    profilePoints.Add(new VelocityPoint(startDistance + availableDistance, v1, constraint));
                }
                else if (situation == 8)
                {
                    var brakeDistance = RampCalculator.CalculateDistanceNeeded(v1, v2, Parameters);
                    if (brakeDistance <= availableDistance)
                    {
                        profilePoints.Add(new VelocityPoint(startDistance + (availableDistance - brakeDistance), v1, constraint));
                        profilePoints.Add(new VelocityPoint(startDistance + availableDistance, v2, constraint));
                    }
                    else
                    {
                        MergeWithPreviousConstraint(constraint, profilePoints, i);
                        return false;
                    }
                }
                else if (situation == 3)
                {
                    var distanceForAcc = RampCalculator.CalculateDistanceNeeded(v0, v1, Parameters);
                    if (distanceForAcc < availableDistance)
                    {
                        profilePoints.Add(new VelocityPoint(startDistance + distanceForAcc, v1, constraint));
                        profilePoints.Add(new VelocityPoint(startDistance + availableDistance, v1, constraint));
                    }
                    else
                    {
                        MergeWithPreviousConstraint(constraint, profilePoints, i);
                    }
                }
                else if (!IterativlyFindSteppedDownVelocity(v0, constraint, v2, availableDistance, startDistance, profilePoints))
                {
                    switch (situation)
                    {
                        case 1:
                        case 4:
                            MergeWithNextConstraint(constraint, i);
                            break;
                        case 2:
                        case 3:
                        case 5:
                        case 6:
                            MergeWithPreviousConstraint(constraint, profilePoints, i);
                            break;
                        case 7:
                            break;
                        case 8:
                            break;
                        default:
                            throw new JointMotionCalculationException($"Unknown situation id {situation}");
                    }
                    return false;
                }

                //var targetVelocity = constraint.MaximumVelocity;

                //if (AreSameVelocities(startVelocity, targetVelocity) || RampCalculator.IsReachable(startVelocity, targetVelocity, availableDistance, Parameters))
                //{
                //    // maximum allowed velocity can be reached
                //    var distanceForFullAcc = RampCalculator.CalculateDistanceNeeded(startVelocity, constraint.MaximumVelocity, Parameters);
                //    if (nextConstraint != null && nextConstraint.MaximumVelocity > constraint.MaximumVelocity)
                //    {
                //        // next constraint allows higher velocity 
                //        // -> drive this constraint until end with constant velocity because it will be accelerated afterwards
                //        profilePoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                //        profilePoints.Add(new VelocityPoint(startDistance + availableDistance, constraint.MaximumVelocity, constraint));
                //    }
                //    else
                //    {
                //        // next constraint is below current constraint -> braking is necessary 
                //        // OR
                //        // no next constraint available -> brake to zero
                //        targetVelocity = nextConstraint?.MaximumVelocity ?? 0.0;
                //        var distanceForBraking = RampCalculator.CalculateDistanceNeeded(constraint.MaximumVelocity, targetVelocity, Parameters);

                //        if (distanceForFullAcc + distanceForBraking < availableDistance)
                //        {
                //            // constant velocity will be reached
                //            profilePoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                //            profilePoints.Add(new VelocityPoint(startDistance + (availableDistance - distanceForBraking), constraint.MaximumVelocity, constraint));
                //            profilePoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                //        }
                //        else if (distanceForFullAcc + distanceForBraking == availableDistance)
                //        {
                //            // constant velocity will not be reached but maximum velocity => exact peak
                //            profilePoints.Add(new VelocityPoint(startDistance + distanceForFullAcc, constraint.MaximumVelocity, constraint));
                //            profilePoints.Add(new VelocityPoint(startDistance + availableDistance, targetVelocity, constraint));
                //        }
                //        else
                //        {
                //            // we can not fully accelerate because we need to brake earlier than possible when only accelerating
                //            // UNTIL I KNOW BETTER: iterative approach
                //            if (constraint.MaximumVelocity >= startVelocity && targetVelocity < startVelocity)
                //            {
                //                MergeWithPreviousConstraint(constraint, profilePoints, ref i);
                //                return false;
                //            }
                //            else
                //            {
                //                if (!IterativlyFindSteppedDownVelocity(startVelocity, constraint, availableDistance, startDistance, targetVelocity, profilePoints, ref i))
                //                {
                //                    return false;
                //                }
                //            }
                //        }
                //    }
                //}
                //else if (nextConstraint != null)
                //{
                //    // maximum allowed velocity can't be reached and there is at least one constraint to follow
                //    var firstStepUp = constraint.MaximumVelocity > startVelocity;
                //    var secondStepUp = nextConstraint.MaximumVelocity > constraint.MaximumVelocity;
                //    if (firstStepUp)
                //    {
                //        if (secondStepUp)
                //        {
                //            MergeWithPreviousConstraint(constraint, profilePoints, ref i);
                //            return false;
                //        }
                //        else
                //        {
                //            var nextReachable = RampCalculator.IsReachable(startVelocity, nextConstraint.MaximumVelocity, availableDistance, Parameters);
                //            if (nextReachable)
                //            {
                //                if (!IterativlyFindSteppedDownVelocity(startVelocity, constraint, availableDistance, startDistance, nextConstraint.MaximumVelocity, profilePoints, ref i))
                //                {
                //                    return false;
                //                }
                //            }
                //            else
                //            {
                //                if (nextConstraint.MaximumVelocity < startVelocity)
                //                {
                //                    MergeWithPreviousConstraint(constraint, profilePoints, ref i);
                //                }
                //                else
                //                {
                //                    MergeWithNextConstraint(constraint, ref i);
                //                }
                //                return false;
                //            }
                //        }
                //    }
                //    else
                //    {
                //        if (!secondStepUp)
                //        {
                //            MergeWithPreviousConstraint(constraint, profilePoints, ref i);
                //            return false;
                //        }
                //        else
                //        {
                //            MergeWithNextConstraint(constraint, ref i);
                //            return false;
                //        }
                //    }
                //}
                //else
                //{
                //    // no constraint any more -> brake to zero
                //    if (!IterativlyFindSteppedDownVelocity(startVelocity, constraint, availableDistance, startDistance, 0.0, profilePoints, ref i))
                //    {
                //        return false;
                //    }
                //}
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

            return true;
        }

        private int GetSituation(double v0, VelocityConstraint constraint, VelocityConstraint nextConstraint)
        {
            var v1 = constraint.MaximumVelocity;
            var v2 = nextConstraint?.MaximumVelocity ?? 0.0;

            if (v1 == v0)
            {
                // situations 7-8
                if (v1 < v2)
                {
                    return 7;
                }
                else
                {
                    return 8;
                }
            }
            else if (v1 > v0)
            {
                // situations 1-3
                if (v2 < v1 && v2 > v0)
                {
                    return 1;
                }
                else if (v2 < v1 && v2 < v0)
                {
                    return 2;
                }
                else
                {
                    return 3;
                }
            }
            else
            {
                // situations 4-6
                if (v2 > v1 && v2 < v0)
                {
                    return 4;
                }
                else if (v2 > v1 && v2 > v0)
                {
                    return 5;
                }
                else
                {
                    return 6;
                }
            }
        }

        /// <summary>
        /// Removes the given constraint and adds its length to the next constraint.
        /// </summary>
        private void MergeWithNextConstraint(VelocityConstraint constraint, int index)
        {
            EffectiveConstraints.RemoveAt(index);
            EffectiveConstraints[index].Start -= constraint.Length;
            EffectiveConstraints[index].Length += constraint.Length;
        }

        /// <summary>
        /// Removes the given constraint and adds its length to the previous constraint 
        /// => the previous constraint is longer now. Enables earlier braking.
        /// </summary>
        private void MergeWithPreviousConstraint(VelocityConstraint constraint, List<VelocityPoint> velocityPoints, int index)
        {
            const double reduceByDistance = 100;
            const double reduceByVelocity = 50;
            if (index == 0)
            {
                // first constraint -> no previous constraint -> try reducing velocity for reachability
                constraint.MaximumVelocity -= reduceByVelocity;
            }
            else
            {
                var previousConstraint = EffectiveConstraints[index - 1];
                if (previousConstraint.MaximumVelocity > constraint.MaximumVelocity && previousConstraint.Length > reduceByDistance)
                {
                    constraint.Start -= reduceByDistance;
                    constraint.Length += reduceByDistance;
                    previousConstraint.Length -= reduceByDistance;
                }
                else if (previousConstraint.MaximumVelocity < constraint.MaximumVelocity && Math.Abs(previousConstraint.MaximumVelocity - constraint.MaximumVelocity) > reduceByVelocity)
                {
                    constraint.MaximumVelocity -= reduceByVelocity;
                }
                else
                {
                    EffectiveConstraints.RemoveAt(index);
                    previousConstraint.Length += constraint.Length;
                    previousConstraint.MaximumVelocity = Math.Min(constraint.MaximumVelocity, previousConstraint.MaximumVelocity);

                    var correspondingConstraint = velocityPoints.Last().CorrespondingConstraint;
                    velocityPoints.RemoveAll(vp => vp.CorrespondingConstraint == correspondingConstraint);
                }
            }
        }

        private bool IterativlyFindSteppedDownVelocity(double lastVelocity, VelocityConstraint constraint, double targetVelocity, double availableDistance, double startDistance, List<VelocityPoint> velocityPoints)
        {
            var targetAccVelocity = constraint.MaximumVelocity;
            var stepDownSize = 5.0;
            while (true)
            {
                if (targetAccVelocity <= lastVelocity)
                {
                    // try with lastVelocity last time because we may have overstepped
                    // that critial point
                    return TryAddVelocityPoints(lastVelocity);
                }
                else if (TryAddVelocityPoints(targetAccVelocity))
                {
                    // successfully found a new targetVelocity which allows
                    // for accelerating and braking
                    return true;
                }
                targetAccVelocity -= stepDownSize;
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