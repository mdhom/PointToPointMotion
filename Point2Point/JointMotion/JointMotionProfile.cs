using System;
using System.Collections.Generic;
using System.Linq;
using Shuttles.Base.Devices.Shuttles.Motion.Ramp;

namespace Point2Point.JointMotion
{
    public class JointMotionProfile
    {
        public RampMotionParameter Parameters { get; }

        public ConstraintsCollection EffectiveConstraints { get; private set; }
        public List<VelocityPoint> VelocityProfilePoints { get; private set; }
        public List<RampCalculationResult> RampResults { get; private set; }
        public List<double> TimesAtVelocityPoints { get; private set; }
        public double TotalDuration { get; private set; }
        public int NumRecalculations { get; }

#if DEBUG
        public List<ConstraintsCollection> EffectiveConstraintsHistory { get; private set; }
#endif

        #region Constructors

        public JointMotionProfile(RampMotionParameter parameters, ConstraintsCollection constraints)
        {
            Parameters = parameters;

            EffectiveConstraints = constraints.GetEffectiveConstraints();

#if DEBUG
            EffectiveConstraintsHistory = new List<ConstraintsCollection>
            {
                new ConstraintsCollection(EffectiveConstraints.Select(ec => ec.Copy()))
            };

            CloseHighTightGapsIteratively();

            while (true)
            {
                EffectiveConstraintsHistory.Add(new ConstraintsCollection(EffectiveConstraints.Select(ec => ec.Copy())));

                if (CalculateProfile())
                {
                    return;
                }
                else
                {
                    // possibility to set brakepoint here
                    NumRecalculations++;
                }
            }
#else
            while (!CalculateProfile())
            {
            }
#endif
        }

        public JointMotionProfile(RampMotionParameter parameters, IEnumerable<VelocityConstraint> constraints)
            : this(parameters, new ConstraintsCollection(constraints))
        {
        }

        public JointMotionProfile(RampMotionParameter parameters, VelocityConstraint constraint, params VelocityConstraint[] constraints)
            : this(parameters, new List<VelocityConstraint>() { constraint }.Concat(constraints))
        {
        }

        #endregion

        #region Public get v/s/satus methods

        /// <summary>
        /// Gets the velocity [mm/s] at the given time [s]
        /// </summary>
        /// <param name="t">Time in seconds within the profile. Must be >= 0 and <= TotalDuration</param>
        /// <returns>Velocity [mm/s] at the given time</returns>
        public double GetV(double t)
        {
            GetStatus(t, out var v, out _);
            return v;
        }

        /// <summary>
        /// Gets the driven distance [mm] at the given time [s]
        /// </summary>
        /// <param name="t">Time in seconds within the profile. Must be >= 0 and <= TotalDuration</param>
        /// <returns>Distance [mm] at the given time</returns>
        public double GetS(double t)
        {
            GetStatus(t, out _, out var s);
            return s;
        }

        /// <summary>
        /// Gets the status at the given time [s]
        /// </summary>
        /// <param name="t">Time in seconds within the profile. Must be >= 0 and <= TotalDuration</param>
        /// <param name="v">Velocity [mm/s] at the given time</param>
        /// <param name="s">Distance [mm] at the given time</param>
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

        #endregion

        private bool CalculateProfile()
        {
            var velocityPoints = new List<VelocityPoint>()
            {
                new VelocityPoint(0,0, null)
            };

            for (var i = 0; i < EffectiveConstraints.Count; i++)
            {
                var constraint = EffectiveConstraints[i];
                var nextConstraint = EffectiveConstraints.ElementAtOrDefault(i + 1);

                var v0 = velocityPoints.Last().Velocity;
                var v1 = constraint.MaximumVelocity;
                var v2 = nextConstraint?.MaximumVelocity ?? 0.0;

                var startDistance = velocityPoints.Max(v => v.Distance);

                var situation = GetSituation(v0, constraint.MaximumVelocity, nextConstraint?.MaximumVelocity ?? 0);

                if (situation == 3)
                {
                    var distanceForAcc = RampCalculator.CalculateDistanceNeeded(v0, v1, Parameters);
                    if (distanceForAcc < constraint.Length)
                    {
                        velocityPoints.Add(new VelocityPoint(startDistance + distanceForAcc, v1, constraint));
                        velocityPoints.Add(new VelocityPoint(startDistance + constraint.Length, v1, constraint));
                    }
                    else
                    {
                        if (MergeWithPreviousConstraint(constraint, i))
                        {
                            RemoveVelocityPointsOfLastConstraint(velocityPoints);
                        }
                        return false;
                    }
                }
                else if (situation == 4 || situation == 5)
                {
                    throw new JointMotionCalculationException($"Situation {situation} must not appear! Something went wrong before");
                }
                else if (situation == 7)
                {
                    velocityPoints.Add(new VelocityPoint(startDistance + constraint.Length, v1, constraint));
                }
                else if (situation == 8)
                {
                    var brakeDistance = RampCalculator.CalculateDistanceNeeded(v1, v2, Parameters);
                    if (brakeDistance <= constraint.Length)
                    {
                        velocityPoints.Add(new VelocityPoint(startDistance + (constraint.Length - brakeDistance), v1, constraint));
                        velocityPoints.Add(new VelocityPoint(startDistance + constraint.Length, v2, constraint));
                    }
                    else
                    {
                        if (MergeWithPreviousConstraint(constraint, i))
                        {
                            RemoveVelocityPointsOfLastConstraint(velocityPoints);
                        }
                        return false;
                    }
                }
                else if (!IterativlyFindSteppedDownVelocity(v0, constraint, v2, startDistance, velocityPoints))
                {
                    switch (situation)
                    {
                        case 1:
                            MergeWithNextConstraint(constraint, i);
                            break;
                        case 2:
                        case 6:
                            if (MergeWithPreviousConstraint(constraint, i))
                            {
                                RemoveVelocityPointsOfLastConstraint(velocityPoints);
                            }
                            break;
                        default:
                            throw new JointMotionCalculationException($"Unhandled situation id {situation}");
                    }
                    return false;
                }
            }

            // remove ProfilePoints with same distance and velocity
            velocityPoints = velocityPoints.DistinctBy(pp => new { pp.Distance, pp.Velocity }).ToList();

            // calculate ramp results and times
            var rampResults = new List<RampCalculationResult>();
            var times = new List<double>();
            var timeSum = 0.0;
            for (var i = 0; i < velocityPoints.Count - 1; i++)
            {
                var pFrom = velocityPoints[i];
                var pTo = velocityPoints[i + 1];

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

            VelocityProfilePoints = velocityPoints;
            RampResults = rampResults;
            TimesAtVelocityPoints = times;
            TotalDuration = timeSum;

            if (double.IsNaN(TotalDuration) || double.IsInfinity(TotalDuration))
            {
                throw new JointMotionCalculationException($"Invalid TotalDuration ({TotalDuration})");
            }

            return true;
        }

        #region GapClosing

        /// <summary>
        /// Iteratively calls the <see cref="CloseHighTightGaps"/> method until it succeeds.
        /// See <see cref="CloseHighTightGaps"/> for further explanation
        /// </summary>
        private void CloseHighTightGapsIteratively()
        {
            while (CloseHighTightGaps())
            {
                // repeat as long as CloseHightTightGaps returns false
            }
        }

        /// <summary>
        /// Removes / closes constraint where the velocity profile will never be
        /// able to enter above the minimnum of velocities (v0, v1 and v2). Can be closed
        /// upfront to save iterations in the more complex CalculateProfile method. 
        /// Attention: for closing all gaps, you need to call the method iteratively, because
        /// new gaps can emerge when existing gaps are closed! Use <see cref="CloseHighTightGapsIteratively" /> method therefore.
        /// </summary>
        /// <returns>True if all gaps were closed, otherwise false.</returns>
        private bool CloseHighTightGaps()
        {
            var highTightGaps = new List<Gap>();
            for (var i = 0; i < EffectiveConstraints.Count; i++)
            {
                var constraint = EffectiveConstraints[i];
                var v0 = i > 0 ? EffectiveConstraints[i - 1].MaximumVelocity : 0;
                var v1 = constraint.MaximumVelocity;
                var v2 = i < EffectiveConstraints.Count - 1 ? EffectiveConstraints[i + 1].MaximumVelocity : 0;

                if (v1 > v0 && v1 > v2)
                {
                    var distanceAcc = RampCalculator.CalculateDistanceNeeded(v0, v1, Parameters);
                    var distanceDec = RampCalculator.CalculateDistanceNeeded(v1, v2, Parameters);
                    if (distanceAcc + distanceDec > constraint.Length)
                    {
                        var distanceAccToV2 = RampCalculator.CalculateDistanceNeeded(v0, v2, Parameters);
                        if (distanceAccToV2 < constraint.Length)
                        {
                            highTightGaps.Add(new Gap(constraint, v0, v1, v2, Gap.ActionType.DoNothing));
                        }
                        else
                        {
                            if (v0 > v2)
                            {
                                highTightGaps.Add(new Gap(constraint, v0, v1, v2, Gap.ActionType.MergeWithPrevious));
                            }
                            else
                            {
                                highTightGaps.Add(new Gap(constraint, v0, v1, v2, Gap.ActionType.MergeWithNext));
                            }
                        }
                    }
                }
            }

            var gapsOrdered = highTightGaps.OrderByDescending(g => g.Constraint.MaximumVelocity);
            foreach (var gap in gapsOrdered)
            {
                switch (gap.Action)
                {
                    case Gap.ActionType.MergeWithPrevious:
                        MergeWithPreviousConstraint(gap.Constraint, EffectiveConstraints.IndexOf(gap.Constraint), true);
                        break;
                    case Gap.ActionType.MergeWithNext:
                        MergeWithNextConstraint(gap.Constraint, EffectiveConstraints.IndexOf(gap.Constraint));
                        break;
                    case Gap.ActionType.DoNothing:
                    default:
                        break;
                }
            }

            return highTightGaps.Any(g => g.Action != Gap.ActionType.DoNothing);
        }

        private class Gap
        {
            public enum ActionType
            {
                MergeWithPrevious,
                MergeWithNext,
                DoNothing,
            }

            public VelocityConstraint Constraint { get; }
            public double v0 { get; }
            public double v1 { get; }
            public double v2 { get; }
            public ActionType Action { get; }

            public Gap(VelocityConstraint constraint, double v0, double v1, double v2, ActionType action)
            {
                Constraint = constraint;
                this.v0 = v0;
                this.v1 = v1;
                this.v2 = v2;
                Action = action;
            }
        }

        #endregion

        /// <summary>
        /// Determines the specific situation, in which the given constraint is located. The situation
        /// is determined by the previous velocity, the constraint velocity and the suceeding velocity (v0, v1, v2).
        /// See ConstraintSituations.pdf for visual explanation.
        /// </summary>
        /// <param name="v0">Incoming velocity at the beginning of the constraint</param>
        /// <param name="v1">MaximumVelocity of the constraint itself</param>
        /// <param name="v2">Outgoing velocity = MaximumVelocity of the next constraint</param>
        /// <returns>An integer between 1 and 8, representing the current situation.</returns>
        private int GetSituation(double v0, double v1, double v2)
        {
            if (v1 == v0)
            {
                // situations 7-8
                return v1 < v2 ? 7 : 8;
            }
            else if (v1 > v0)
            {
                // situations 1-3
                if (v2 < v1)
                {
                    return v2 >= v0 ? 1 : 2;
                }
                else
                {
                    return 3;
                }
            }
            else
            {
                // situations 4-6
                if (v2 > v1)
                {
                    return v2 <= v0 ? 4 : 5;
                }
                else
                {
                    // v2 < v1
                    return 6;
                }
            }
        }

        /// <summary>
        /// Removes the given constraint, moves the start of the next constraint to the start of the
        /// given constraint and adds its length to the next constraint.
        /// </summary>
        private void MergeWithNextConstraint(VelocityConstraint constraint, int index)
        {
            EffectiveConstraints.RemoveAt(index);
            EffectiveConstraints[index].Start -= constraint.Length;
            EffectiveConstraints[index].Length += constraint.Length;
        }

        /// <summary>
        /// (Iteratively) merges the given constraint with the previous constraint the the lower MaxVel of both.
        /// </summary>
        /// <param name="constraint">Constraint to merge with its previous constraint</param>
        /// <param name="index">Index of the given constraint within the EffectiveConstraints list</param>
        /// <param name="forceMerge">Forces a complete merge without iteratively approaching to the previous constraint</param>
        /// <returns>True if constraint was completely merged, otherwise false</returns>
        private bool MergeWithPreviousConstraint(VelocityConstraint constraint, int index, bool forceMerge = false)
        {
            const double reduceByDistance = 100;    // mm
            const double reduceByVelocity = 50;     // mm/s

            if (index == 0)
            {
                // first constraint -> no previous constraint -> try reducing velocity for reachability
                constraint.ReduceBy(reduceByVelocity);
            }
            else
            {
                var previousConstraint = EffectiveConstraints[index - 1];
                if (!forceMerge && previousConstraint > constraint && previousConstraint.Length > reduceByDistance)
                {
                    // The previous constraint is higher than the given constraint. As it is not allowed
                    // to increase the MaxVel of a constraint, the MaxVel of the previous constraint must be reduced.
                    // Because the previous constraint may be a very long one, a complete merge at one time could waste
                    // precious "high velocity time". Therefore, the merging is done iteratively.
                    constraint.Start -= reduceByDistance;
                    constraint.Length += reduceByDistance;
                    previousConstraint.Length -= reduceByDistance;
                }
                else if (!forceMerge && previousConstraint < constraint && Math.Abs(previousConstraint - constraint) > reduceByVelocity)
                {
                    // The previous constraint is below the given constraint. Therefore, the given constraints MaxVelo
                    // must be reduced. Because that could be a waste of "high velocity time", this is done interatively.
                    constraint.ReduceBy(reduceByVelocity);
                }
                else
                {
                    // Either foreMerge is true or the constraint which should be reduced is not long / high enough anymore
                    // -> now completely merge the constraints by removing the given constraint and adding its length to the previous constraint
                    // To be safe, the minimum of the both MaxVels is taken
                    EffectiveConstraints.RemoveAt(index);
                    previousConstraint.Length += constraint.Length;
                    previousConstraint.MaximumVelocity = Math.Min(constraint.MaximumVelocity, previousConstraint.MaximumVelocity);
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// VelocityPoints for the previous constraint were added, remove those because the constraint does not exist anymore.
        /// </summary>
        /// <param name="velocityPoints">List of all previously added velocityPoints</param>
        private void RemoveVelocityPointsOfLastConstraint(List<VelocityPoint> velocityPoints)
        {
            var correspondingConstraint = velocityPoints.Last().CorrespondingConstraint;
            velocityPoints.RemoveAll(vp => vp.CorrespondingConstraint == correspondingConstraint);
        }

        /// <summary>
        /// If the constraint's MaxVel is to high to be accelerated to from v0 and then be
        /// decellerated from to v2 (all within the available distance), this method tries to
        /// iteratively reduce the MaxVel of the constraint, until it can be reached from 
        /// acceleration as well from decelleration side.
        /// </summary>
        /// <param name="v0">Velocity at which the given constraint is entered</param>
        /// <param name="constraint">The constraint for which a solution is searched</param>
        /// <param name="v2">Velocity at which the constraint will be left</param>
        /// <param name="availableDistance">Distance which is available for reaching and leaving the constraint</param>
        /// <param name="startDistance">Absolute distance from the beginning of the profile until the start of the constraint</param>
        /// <param name="velocityPoints">List of previously added velocityPoints</param>
        /// <returns>True if stepped down velocity was found, otherwise false</returns>
        private bool IterativlyFindSteppedDownVelocity(double v0, VelocityConstraint constraint, double v2, double startDistance, List<VelocityPoint> velocityPoints)
        {
            const double stepDownSize = 5.0;

            var v1 = constraint.MaximumVelocity;
            var limitVelocity = Math.Max(v0, v2);
            while (v1 > limitVelocity)
            {
                if (TryAddVelocityPoints(v1))
                {
                    // successfully found a new targetVelocity which allows
                    // for accelerating and braking
                    return true;
                }

                v1 -= stepDownSize;
            }

            // failed to add velocityPoints with any intermediate velocity
            // -> try with lastVelocity last time because we may have overstepped that critial point
            return TryAddVelocityPoints(limitVelocity);

            bool TryAddVelocityPoints(double v)
            {
                var distanceForSDAcc = RampCalculator.CalculateDistanceNeeded(v0, v, Parameters);
                var distanceForBrakingFromSD = RampCalculator.CalculateDistanceNeeded(v, v2, Parameters);
                if (distanceForSDAcc + distanceForBrakingFromSD < constraint.Length)
                {
                    // constant velocity will be reached
                    velocityPoints.Add(new VelocityPoint(startDistance + distanceForSDAcc, v, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + (constraint.Length - distanceForBrakingFromSD), v, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + constraint.Length, v2, constraint));
                    return true;
                }
                else if (distanceForSDAcc + distanceForBrakingFromSD == constraint.Length)
                {
                    // constant velocity will not be reached but maximum velocity => exact peak
                    velocityPoints.Add(new VelocityPoint(startDistance + distanceForSDAcc, v, constraint));
                    velocityPoints.Add(new VelocityPoint(startDistance + constraint.Length, v2, constraint));
                    return true;
                }

                return false;
            }
        }
    }
}