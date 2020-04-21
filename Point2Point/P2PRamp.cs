using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point
{
    public class P2PRamp
    {
        private readonly double _j;
        private readonly double _aMax;

        private P2PRamp(double jMax, double aMax)
        {
            _j = jMax;
            _aMax = aMax;
        }

        public static double GetReachableVelocity(double distance, double v0, double jMax, double aMax)
        {
            var calculator = new P2PRamp(jMax, aMax);
            return calculator.GetReachableVelocity(distance, v0);
        }

        private double GetReachableVelocity(double distance, double v0)
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
            var dForAMax = _aMax / _j;
            var d1 = dForAMax;
            var d2 = 0.0;
            CalculateProfile(v0, d1, d2, out var v3, out var s3);

            if (s3 == distance)
            {
                // The previously calculated profile covers exactly the given distance.
                // The reachable velocity is exactly the velocity reached with this profile.
                return v3;
            }
            else if (s3 < distance)
            {
                // The previously calculated profile does not even cover the given distance
                // => Phase 2 with constant acceleration is necessary
                // => d2 > 0
                CalculateWithConstantA(distance, v0, d1, out v3, out s3);
                if (!IsDistanceValid(s3))
                {
                    throw new InvalidOperationException($"Invalid calculated distance {s3}, target would be {distance}");
                }

                return v3;
            }
            else
            {
                // The previously calculated profile overshoots the given distance.
                // To stay within the given distance, aMax obviously must not be reached.
                // => d1 < dForAMax!!
                CalculateWithANotReached(distance, v0, out v3, out s3);
                if (!IsDistanceValid(s3))
                {
                    throw new InvalidOperationException($"Invalid calculated distance {s3}, target would be {distance}");
                }

                return v3;
            }

            bool IsDistanceValid(double calculatedDistance)
                => Math.Abs(calculatedDistance - distance) < 1e-8;
        }

        private void CalculateWithANotReached(double distance, double v0, out double v3, out double s3)
        {
            var j_2 = _j * _j;
            var j_3 = j_2 * _j;
            var j_4 = j_3 * _j;

            // Wolfram Alphas output splitted in single parts for better readability
            // https://www.wolframalpha.com/input/?i=2vx%2Bjx%5E3-s%3D0
            var p1 = Math.Sqrt(27 * j_4 * distance * distance + 32 * j_3 * v0 * v0 * v0);
            var p2 = CubeRoot.Get(9 * j_2 * distance + Math.Sqrt(3) * p1);
            var p3 = CubeRoot.Get(2) * Math.Pow(3, 2.0 / 3) * _j;
            var p4 = 2 * CubeRoot.Get(2.0 / 3) * v0;
            var p5 = CubeRoot.Get(9 * j_2 * distance + Math.Sqrt(3) * p1);

            var d1 = p2 / p3 - p4 / p5;

            CalculateProfile(v0, d1, 0, out v3, out s3);
        }

        private void CalculateWithConstantA(double distance, double v0, double d1, out double v3, out double s3)
        {
            // solving quadratic equation using "Mitternachtsformel"
            var mnfA = 0.5 * _j * d1;
            var mnfB = v0 + 1.5 * _j * d1 * d1;
            var mnfC = 2 * v0 * d1 + _j * d1 * d1 * d1 - distance;

            // "Mitternachtsformel" provides two results
            var d2_1 = (-mnfB + Math.Sqrt(mnfB * mnfB - 4 * mnfA * mnfC)) / (2 * mnfA);
            var d2_2 = (-mnfB - Math.Sqrt(mnfB * mnfB - 4 * mnfA * mnfC)) / (2 * mnfA);

            // select non-negative result
            var d2 = d2_1 < 0 ? d2_2 : d2_1;

            // calculate profile with the newly calculated d2
            CalculateProfile(v0, d1, d2, out v3, out s3);
        }

        #region Calculations from P2PCalculator, extended by v0

        private void CalculateProfile(double v0, double d1, double d2, out double v3, out double s3)
        {
            var t1 = d1;
            var t2 = t1 + d2;
            var t3 = t2 + d1;
            GetStatus1(t1, v0, out var j1, out var a1, out var v1, out var s1);
            GetStatus2(t2, t1, a1, v1, s1, out var j2, out var a2, out var v2, out var s2);
            GetStatus3(t3, t2, a2, v2, s2, out var j3, out var a3, out v3, out s3);
        }

        private void GetStatus1(double t1, double v0, out double j, out double a, out double v, out double s)
        {
            j = this._j;
            a = this._j * t1;
            v = v0 + 0.5 * this._j * t1 * t1;
            s = v0 * t1 + this._j / 6 * t1 * t1 * t1;
        }

        private void GetStatus2(double t2, double t1, double a1, double v1, double s1, out double j, out double a, out double v, out double s)
        {
            j = 0;
            a = a1;
            v = v1 + a1 * (t2 - t1);
            s = s1 + v1 * (t2 - t1) + 0.5 * a1 * (t2 - t1) * (t2 - t1);
        }

        private void GetStatus3(double t3, double t2, double a2, double v2, double s2, out double j, out double a, out double v, out double s)
        {
            var tPhase = t3 - t2;
            var tPhase2 = tPhase * tPhase;
            var tPhase3 = tPhase2 * tPhase;
            j = -this._j;
            a = a2 - this._j * tPhase;
            v = v2 + a2 * tPhase + 0.5 * -this._j * tPhase2;
            s = s2 + v2 * tPhase + 0.5 * a2 * tPhase2 + -this._j / 6 * tPhase3;
        }

        #endregion
    }
}
