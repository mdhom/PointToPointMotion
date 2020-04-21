using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point
{
    public class P2PRamp
    {
        private readonly double _jMax;
        private readonly double _aMax;
        private readonly double _v0;

        private P2PRamp(double jMax, double aMax, double v0)
        {
            _jMax = jMax;
            _aMax = aMax;
            _v0 = v0;
        }

        public static double GetReachableVelocity(double distance, double v0, double jMax, double aMax)
        {
            var calculator = new P2PRamp(jMax, aMax, v0);
            return calculator.GetReachableVelocity(distance);
        }

        private double GetReachableVelocity(double distance)
        {
            // Phase 1: positive jerk, lineary increasing acceleration, quadratic increasing velocity
            // Phase 2: zero jerk, constant acceleration, lineary increasing velocity
            // Phase 3: negative jerk, lineary decreasing acceleration, quadratic increasing velocity
            //
            // d1, d2, d3 means "duration" of the specific phase
            // Assumption: d1 = d3 because start- and end-acceleration should both be zero, so 
            // same time for positive as well as negative jerk is needed!
            //
            // Question 1: how big need d1 (=d3) and d2 be to exactly cover the given distance (s3 must be equal to distance)?
            // Question 2: how big is the reached velocity given that profile

            // Let's look where we land in case we have no constant a (d2=0)
            // but completely reach aMax.
            var dForAMax = _aMax / _jMax;
            var d1 = dForAMax;
            var d2 = 0.0;
            CalculateEndValues(d1, d2, _jMax, out _, out var s3);

            if (s3 < distance)
            {
                // The previously calculated profile does not even cover the given distance
                // => Phase 2 with constant acceleration is necessary
                // => d2 > 0
                (d1, d2) = CalculateTimesWithConstantA(distance, dForAMax);
            }
            else if  (s3 > distance)
            {
                // The previously calculated profile overshoots the given distance.
                // To stay within the given distance, aMax obviously must not be reached.
                // => d1 < dForAMax!!
                (d1, d2) = CalculateTimesWithANotReached(distance);
            }

            CalculateEndValues(d1, d2, _jMax, out var v3, out s3);

            if (Math.Abs(s3 - distance) > 1e-8)
            {
                throw new InvalidOperationException($"Invalid calculated distance {s3}, target would be {distance}");
            }

            return v3;
        }

        private (double d1, double d2) CalculateTimesWithANotReached(double distance)
        {
            var j_2 = _jMax * _jMax;
            var j_3 = j_2 * _jMax;
            var j_4 = j_3 * _jMax;

            // Wolfram Alphas output splitted in single parts for better readability
            // https://www.wolframalpha.com/input/?i=2vx%2Bjx%5E3-s%3D0
            var p1 = Math.Sqrt(27 * j_4 * distance * distance + 32 * j_3 * _v0 * _v0 * _v0);
            var p2 = CubeRoot.Get(9 * j_2 * distance + Math.Sqrt(3) * p1);
            var p3 = CubeRoot.Get(2) * Math.Pow(3, 2.0 / 3) * _jMax;
            var p4 = 2 * CubeRoot.Get(2.0 / 3) * _v0;
            var p5 = CubeRoot.Get(9 * j_2 * distance + Math.Sqrt(3) * p1);

            var d1 = p2 / p3 - p4 / p5;

            return (d1, 0);
        }

        private (double d1, double d2) CalculateTimesWithConstantA(double distance, double d1)
        {
            // solving quadratic equation using "Mitternachtsformel"
            var mnfA = 0.5 * _jMax * d1;
            var mnfB = _v0 + 1.5 * _jMax * d1 * d1;
            var mnfC = 2 * _v0 * d1 + _jMax * d1 * d1 * d1 - distance;

            // "Mitternachtsformel" provides two results
            var d2_1 = (-mnfB + Math.Sqrt(mnfB * mnfB - 4 * mnfA * mnfC)) / (2 * mnfA);
            var d2_2 = (-mnfB - Math.Sqrt(mnfB * mnfB - 4 * mnfA * mnfC)) / (2 * mnfA);

            // select non-negative result
            var d2 = d2_1 < 0 ? d2_2 : d2_1;

            return (d1, d2);
        }

        private void CalculateEndValues(double d1, double d2, double jMax, out double v3, out double s3)
        {
            GetStatus(d1 + d2 + d1, _v0, d1, d2, jMax, out _, out _, out v3, out s3);
        }

        #region Calculations from P2PCalculator, extended by v0

        private static void GetStatus(double t, double v0, double d1, double d2, double jMax, out double j, out double a, out double v, out double s)
        {
            var t1 = d1;
            var t2 = t1 + d2;
            var t3 = t2 + d1;

            if (t <= t1)
            {
                GetStatus1(t, v0, jMax, out j, out a, out v, out s);
            }
            else
            {
                GetStatus1(t1, v0, jMax, out var j1, out var a1, out var v1, out var s1);

                if (t <= t2)
                {
                    GetStatus2(t, t1, a1, v1, s1, out j, out a, out v, out s);
                }
                else
                {
                    GetStatus2(t2, t1, a1, v1, s1, out var j2, out var a2, out var v2, out var s2);

                    if (t <= t3)
                    {
                        GetStatus3(t, t2, a2, v2, s2, jMax, out j, out a, out v, out s);
                    }
                    else
                    {
                        throw new ArgumentOutOfRangeException($"Given time t must be lower than t3 which is {t3:N2} at the moment");
                    }
                }
            }
        }

        private static void GetStatus1(double t, double v0, double jMax, out double j, out double a, out double v, out double s)
        {
            j = jMax;
            a = j * t;
            v = v0 + 0.5 * j * t * t;
            s = v0 * t + j / 6 * t * t * t;
        }

        private static void GetStatus2(double t, double t1, double a1, double v1, double s1, out double j, out double a, out double v, out double s)
        {
            j = 0;
            a = a1;
            v = v1 + a1 * (t - t1);
            s = s1 + v1 * (t - t1) + 0.5 * a1 * (t - t1) * (t - t1);
        }

        private static void GetStatus3(double t, double t2, double a2, double v2, double s2, double jMax, out double j, out double a, out double v, out double s)
        {
            var tPhase = t - t2;
            var tPhase2 = tPhase * tPhase;
            var tPhase3 = tPhase2 * tPhase;
            j = -jMax;
            a = a2 - jMax * tPhase;
            v = v2 + a2 * tPhase + 0.5 * -jMax * tPhase2;
            s = s2 + v2 * tPhase + 0.5 * a2 * tPhase2 + -jMax / 6 * tPhase3;
        }

        #endregion
    }
}
