using System;
using System.Collections.Generic;
using System.Linq;

namespace Point2Point.JointMotion
{
    public class JointMotionProfile
    {
        private const double _epsilon = 0.000000001;

        private readonly P2PParameters _parameters;
        public List<VelocityConstraint> EffectiveConstraints { get; }
        public List<VelocityPoint> VelocityPoints { get; }
        public List<double> TimesAtVelocityPoints { get; }

        public double TotalDuration => TimesAtVelocityPoints.Last();

        public JointMotionProfile(ConstraintsCollection constraints, P2PParameters parameters)
        {
            EffectiveConstraints = constraints.GetEffectiveConstraints();
            _parameters = parameters;

            VelocityPoints = CalculateProfile();

            TimesAtVelocityPoints = CalculateTimesAtVelocityPoints();
        }

        public void GetStatus(double t, out double v, out double s)
        {
            try
            {
                if (t > TimesAtVelocityPoints.Last())
                {
                    v = 0;
                    s = 0;
                    return;
                }

                var pointToIndex = TimesAtVelocityPoints.FindIndex(tAtPoint => tAtPoint > t) + 1;
                var pointFromIndex = pointToIndex - 1;

                var pointFrom = VelocityPoints[pointFromIndex];
                var pointTo = VelocityPoints[pointToIndex];

                var tFrom = TimesAtVelocityPoints.ElementAtOrDefault(pointFromIndex - 1);
                var tTo = TimesAtVelocityPoints.ElementAtOrDefault(pointToIndex - 1);

                var distance = pointTo.Distance - pointFrom.Distance;
                if (pointFrom.Velocity == pointTo.Velocity)
                {
                    v = pointFrom.Velocity;
                    s = pointFrom.Distance + (t - tFrom) * v;
                }
                else
                {
                    var calc = new P2PCalculator(distance * 2, _parameters.JerkMax, _parameters.AccelerationMax, Math.Abs(pointFrom.Velocity - pointTo.Velocity));
                    if (pointFrom.Velocity < pointTo.Velocity)
                    {
                        // acceleration
                        calc.GetStatus(t - tFrom, out _, out _, out v, out s);
                        s += pointFrom.Distance;
                    }
                    else
                    {
                        // deceleration -> invert!
                        calc.GetStatus(calc.t3 - (t - tFrom), out _, out _, out v, out s);
                        s = pointTo.Distance - s;
                    }

                    v += Math.Min(pointFrom.Velocity, pointTo.Velocity);
                }
            }
            catch (ArgumentOutOfRangeException)
            {
                v = 0;
                s = 0;
            }
        }

        private List<double> CalculateTimesAtVelocityPoints()
        {
            var times = new List<double>();
            for (int i = 1; i < VelocityPoints.Count; i++)
            {
                var pointFrom = VelocityPoints[i - 1];
                var pointTo = VelocityPoints[i];

                if (pointFrom.Velocity == pointTo.Velocity)
                {
                    // constant motion
                    times.Add(times.LastOrDefault() + (pointTo.Distance - pointFrom.Distance) / pointTo.Velocity);
                }
                else
                {
                    // acc-/deceleration
                    var diffVelo = Math.Abs(pointTo.Velocity - pointFrom.Velocity);
                    var timeForAccDec = P2PCalculator.CalculateTimeForAccDec(diffVelo, _parameters.JerkMax, _parameters.AccelerationMax);
                    times.Add(times.LastOrDefault() + timeForAccDec);
                }
            }

            return times;
        }

        private List<VelocityPoint> CalculateProfile()
        {
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
                var lastVelocity = profilePoints.Last().Velocity;
                var maxReachableVelocity = P2PRamp.GetReachableVelocity(availableDistance, lastVelocity, _parameters.JerkMax, _parameters.AccelerationMax);
                if (maxReachableVelocity > constraint.MaximumVelocity)
                {
                    // maximum allowed velocity can be reached
                    var distanceForFullAcc = P2PCalculator.CalculateDistanceForAcceleration(lastVelocity, constraint.MaximumVelocity, _parameters);
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
                        var distanceForBraking = P2PCalculator.CalculateDistanceForAcceleration(constraint.MaximumVelocity, targetVelocity, _parameters);

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

            for (int i = 0; i < profilePoints.Count - 1; i++)
            {
                if (profilePoints[i].Distance == profilePoints[i + 1].Distance)
                {
                    if (profilePoints[i].Velocity == profilePoints[i + 1].Velocity)
                    {
                        profilePoints.RemoveAt(i);
                        i--;
                    }
                    else
                    {
                        //SPRUNG!!!
                    }
                }
            }

            return profilePoints;
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
                var velocityDifferenceToSDVelo = Math.Abs(targetVel - lastVelocity);
                var distanceForSDAcc = P2PCalculator.CalculateDistanceForAcceleration(lastVelocity, targetVel, _parameters);
                var velocityDifferenceToTargetVelocityFromSD = Math.Abs(targetVel - targetVelocity);
                var distanceForBrakingFromSD = P2PCalculator.CalculateDistanceForAcceleration(targetVel, targetVelocity, _parameters);
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