using System;

namespace Point2Point
{
#pragma warning disable IDE1006 // Naming Styles
#pragma warning disable IDE0059 // Unnecessary assignments
    public class P2PCalculator
    {
        public P2PParameters Parameters { get; }

        public double JerkMax => Parameters.JerkMax;
        public double AccelerationMax => Parameters.AccelerationMax;
        public double VelocityMax => Parameters.VelocityMax;

        public double t1 { get; }
        public double t2 { get; }
        public double t3 { get; }
        public double t4 { get; }
        public double t5 { get; }
        public double t6 { get; }
        public double t7 { get; }
        public int TrajectoryInstanceCase { get; }

        public double TotalTime => t7;

        public P2PCalculator(double s, P2PParameters parameters)
        {
            Parameters = parameters;

            if (JerkMax < 0)
            {
                throw new ArgumentOutOfRangeException($"Jerk must be positive");
            }
            else if (AccelerationMax < 0)
            {
                throw new ArgumentOutOfRangeException($"Acceleration must be positive");
            }
            else if (VelocityMax < 0)
            {
                throw new ArgumentOutOfRangeException($"Velocity must be positive");
            }

            TrajectoryInstanceCase = GetTrajectoryInstance(s);

            CalculateTimes(s, TrajectoryInstanceCase, out var tj, out var ta, out var tv);

            t1 = tj;
            t2 = ta;
            t3 = ta + tj;
            t4 = tv;
            t5 = tv + tj;
            t6 = tv + ta;
            t7 = tv + tj + ta;
        }

        public P2PCalculator(double s, double jerkMax, double accelerationMax, double velocityMax)
            : this(s, new P2PParameters(jerkMax, accelerationMax, velocityMax))
        {
        }

        public void GetStatus(double t, out double j, out double a, out double v, out double s)
        {
            if (t <= t1)
            {
                GetStatus1(t, out j, out a, out v, out s);
            }
            else
            {
                GetStatus1(t1, out var j1, out var a1, out var v1, out var s1);

                if (t <= t2)
                {
                    GetStatus2(t, a1, v1, s1, out j, out a, out v, out s);
                }
                else
                {
                    GetStatus2(t2, a1, v1, s1, out var j2, out var a2, out var v2, out var s2);

                    if (t <= t3)
                    {
                        GetStatus3(t, a2, v2, s2, out j, out a, out v, out s);
                    }
                    else
                    {
                        GetStatus3(t3, a2, v2, s2, out var j3, out var a3, out var v3, out var s3);

                        if (t <= t4)
                        {
                            GetStatus4(t, v3, s3, out j, out a, out v, out s);
                        }
                        else
                        {
                            GetStatus4(t4, v3, s3, out var j4, out var a4, out var v4, out var s4);

                            if (t <= t5)
                            {
                                GetStatus5(t, v4, s4, out j, out a, out v, out s);
                            }
                            else
                            {
                                GetStatus5(t5, v4, s4, out var j5, out var a5, out var v5, out var s5);

                                if (t <= t6)
                                {
                                    GetStatus6(t, a5, v5, s5, out j, out a, out v, out s);
                                }
                                else
                                {
                                    GetStatus6(t6, a5, v5, s5, out var j6, out var a6, out var v6, out var s6);

                                    if (t <= t7)
                                    {
                                        GetStatus7(t, a6, v6, s6, out j, out a, out v, out s);
                                    }
                                    else
                                    {
                                        throw new ArgumentOutOfRangeException($"Given time t must be lower than t7 which is {t7:N2} at the moment");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        private void GetStatus1(double t, out double j, out double a, out double v, out double s)
        {
            j = JerkMax;
            a = JerkMax * t;
            v = 0.5 * JerkMax * t * t;
            s = JerkMax / 6 * t * t * t;
        }

        private void GetStatus2(double t, double a1, double v1, double s1, out double j, out double a, out double v, out double s)
        {
            j = 0;
            a = a1;
            v = v1 + a1 * (t - t1);
            s = s1 + v1 * (t - t1) + 0.5 * a1 * (t - t1) * (t - t1);
        }

        private void GetStatus3(double t, double a2, double v2, double s2, out double j, out double a, out double v, out double s)
        {
            var tPhase = t - t2;
            var tPhase2 = tPhase * tPhase;
            var tPhase3 = tPhase2 * tPhase;
            j = -JerkMax;
            a = a2 - JerkMax * tPhase;
            v = v2 + a2 * tPhase + 0.5 * -JerkMax * tPhase2;
            s = s2 + v2 * tPhase + 0.5 * a2 * tPhase2 + -JerkMax / 6 * tPhase3;
        }

        private void GetStatus4(double t, double v3, double s3, out double j, out double a, out double v, out double s)
        {
            var tPhase = t - t3;
            j = 0;
            a = 0;
            v = v3;
            s = s3 + v3 * tPhase;
        }

        private void GetStatus5(double t, double v4, double s4, out double j, out double a, out double v, out double s)
        {
            var tPhase = t - t4;
            var tPhase2 = tPhase * tPhase;
            var tPhase3 = tPhase2 * tPhase;
            j = -JerkMax;
            a = -JerkMax * tPhase;
            v = v4 + 0.5 * -JerkMax * tPhase2;
            s = s4 + v4 * tPhase + -JerkMax / 6 * tPhase3;
        }

        private void GetStatus6(double t, double a5, double v5, double s5, out double j, out double a, out double v, out double s)
        {
            var tPhase = t - t5;
            var tPhase2 = tPhase * tPhase;
            j = 0;
            a = a5;
            v = v5 - AccelerationMax * tPhase;
            s = s5 + v5 * tPhase + 0.5 * a5 * tPhase2;
        }

        private void GetStatus7(double t, double a6, double v6, double s6, out double j, out double a, out double v, out double s)
        {
            var tPhase = t - t6;
            var tPhase2 = tPhase * tPhase;
            var tPhase3 = tPhase2 * tPhase;
            j = JerkMax;
            a = a6 + JerkMax * tPhase;
            v = v6 + a6 * tPhase + 0.5 * JerkMax * tPhase2;
            s = s6 + v6 * tPhase + 0.5 * a6 * tPhase2 + JerkMax / 6 * tPhase3;
        }

        private int GetTrajectoryInstance(double s)
        {
            // Case 1: a_max reached, v_max reached and constant for a time                             
            //          (s = 100,   j = 2000, a = 500,  vMax = 120)
            // Case 2: a_max not reached, v_max not reached                                             
            //          (s = 15000, j = 2000, a = 5500, vMax = 20500)
            // Case 3: a_max not reached, v_max reached and constant for a time                         
            //          (s = 15000, j = 2000, a = 5500, vMax = 2500)
            // Case 4: a_max reached, v_max not reached                                                 
            //          (s = 57,    j = 2000, a = 500,  vMax = 120)
            // Case 5: a_max reached and constant for a time, v_max reached and constant for a time     
            //          (s = 15000, j = 2000, a = 500,  vMax = 2500)
            // Case 6: a_max reached and constant for a time, v_max not reached                         
            //          (s = 15000, j = 2000, a = 500,  vMax = 20500)

            var v_a = AccelerationMax * AccelerationMax / JerkMax;
            var s_a = 2 * AccelerationMax * AccelerationMax * AccelerationMax / (JerkMax * JerkMax);
            double s_v;
            if (VelocityMax * JerkMax < AccelerationMax * AccelerationMax)
            {
                s_v = VelocityMax * 2 * Math.Sqrt(VelocityMax / JerkMax);
            }
            else
            {
                s_v = VelocityMax * (VelocityMax / AccelerationMax + AccelerationMax / JerkMax);
            }

            if (VelocityMax < v_a && s > s_a)
            {
                return 1;
            }
            else if (VelocityMax > v_a && s < s_a)
            {
                return 2;
            }
            else if (VelocityMax < v_a && s < s_a && s > s_v)
            {
                return 3;
            }
            else if (VelocityMax < v_a && s < s_a && s < s_v)
            {
                return 4;
            }
            else if (VelocityMax > v_a && s > s_a && s > s_v)
            {
                return 5;
            }
            else if (VelocityMax > v_a && s > s_a && s < s_v)
            {
                return 6;
            }
            else
            {
                throw new InvalidOperationException("Undetected case");
            }
        }

        private void CalculateTimes(double s, int trajectoryInstance, out double tj, out double ta, out double tv)
        {
            switch (trajectoryInstance)
            {
                case 1:
                case 3:
                    tj = Math.Sqrt(VelocityMax / JerkMax);
                    ta = tj;
                    tv = s / VelocityMax;
                    return;
                case 2:
                case 4:
                    tj = ThirdRoot(s / (2 * JerkMax));
                    ta = tj;
                    tv = 2 * tj;
                    return;
                case 5:
                    tj = AccelerationMax / JerkMax;
                    ta = VelocityMax / AccelerationMax;
                    tv = s / VelocityMax;
                    return;
                case 6:
                    tj = AccelerationMax / JerkMax;
                    ta = 0.5 * (Math.Sqrt((4 * s * JerkMax * JerkMax + AccelerationMax * AccelerationMax * AccelerationMax) / (AccelerationMax * JerkMax * JerkMax)) - AccelerationMax / JerkMax);
                    tv = ta + tj;
                    return;
                default:
                    throw new ArgumentOutOfRangeException($"TrajectoryInstance must be between 1 and 6");
            }
        }

        private static double ThirdRoot(double x)
        {
            return Math.Pow(x, 1.0 / 3.0);
        }

        public double GetTotalDistance()
        {
            GetStatus(t7, out _, out _, out _, out var s);
            return s;
        }

        public double CalculateMaximumReachedVelocity()
        {
            GetStatus(t3, out _, out _, out var v, out _);
            return v;
        }

        /// <summary>
        /// Calculates maximum reachable velocity within the given distance.
        /// Constant (or no) motion (a=0) required as initial situation. 
        /// Assumption: reached velocity will be constant (a=0)
        /// </summary>
        /// <param name="distance"></param>
        /// <returns></returns>
        public static double CalculateMaximumReachableVelocity(double distance, P2PParameters parameters)
        {
            // uses assumption that profile is always symetrical, so doubling the distance
            // should make sure, that phase 4 (constant velocity) is reached
            var calc = new P2PCalculator(distance * 2, parameters);
            return calc.CalculateMaximumReachedVelocity();
        }

        public static double GetDistanceForFullAcceleration(double vMax, P2PParameters parameters)
        {
            // uses assumption that profile is always symetrical, so doubling the distance
            // should make sure, that phase 4 (constant velocity) is reached
            var parametersModified = new P2PParameters(parameters.JerkMax, parameters.AccelerationMax, vMax);
            var calc = new P2PCalculator(double.MaxValue, parametersModified);
            calc.GetStatus(calc.t3, out _, out _, out _, out var s);
            return s;
        }
    }
#pragma warning restore IDE1006 // Naming Styles
#pragma warning restore IDE0059 // Unnecessary assignments
}
