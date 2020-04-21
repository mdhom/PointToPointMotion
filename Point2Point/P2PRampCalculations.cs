using System;
using System.Collections.Generic;
using System.Numerics;
using Shuttles.Base.Devices.Shuttles.Motion.Ramp;

namespace Point2Point
{
    public class P2PRamp
    {
        private readonly double _jMaxDirected;
        private readonly double _v0;
        private readonly double _d1;
        private readonly double _d2;

        public P2PRamp(double distance, double vFrom, double vTo, double jMax, double aMax)
        {
            _jMaxDirected = P2PRampCalculations.GetDirectedJerk(jMax, vFrom, vTo);
            _v0 = vFrom;

            var calc = new P2PRampCalculations(_jMaxDirected, aMax, vFrom);
            //TODO find error in calc.CalculateTimes:  WHY DOES THIS NOT WORK WHEN vTo=0????
            //(_d1, _d2) = calc.CalculateTimes(distance, vTo);
            // Until solved, i take the RampCalculator from Mats:
            var result = RampCalculator.Calculate(0, vFrom, vTo, new RampMotionParameter(new P2PParameters(jMax, aMax, 1000)));
            _d1 = result.t1;
            _d2 = result.t2;

            calc.CalculateEndValues(_d1, _d2, _jMaxDirected, out var v3, out var s3);

            if (Math.Abs(v3 - vTo) > 1e-8)
            {
                throw new InvalidOperationException($"Calculated target velocity ({v3:N3}mm/s) differs from vTo ({vTo:N3}mm/s)");
            }

            if (Math.Abs(s3 - distance) > 1e-8)
            {
                throw new InvalidOperationException($"Calculated distance ({s3:N3}mm) differs from distance ({distance:N3}mm)");
            }
        }

        public void GetStatus(double t, out double j, out double a, out double v, out double s)
        {
            P2PRampCalculations.GetStatus(t, _v0, _d1, _d2, _jMaxDirected, out j, out a, out v, out s);
        }
    }

    public class P2PRampCalculations
    {
        private readonly double _jMax;
        private readonly double _aMax;
        private readonly double _v0;

        internal P2PRampCalculations(AccDecDirection direction, double jMax, double aMax, double v0)
        {
            _jMax = GetDirectedJerk(jMax, direction);
            _aMax = aMax;
            _v0 = v0;
        }

        internal P2PRampCalculations(double jMaxDirected, double aMax, double v0)
        {
            _jMax = jMaxDirected;
            _aMax = aMax;
            _v0 = v0;
        }

        public static AccDecDirection GetAccDecDirection(double vFrom, double vTo)
        {
            if (vFrom == vTo)
                return AccDecDirection.Constant;
            else if (vFrom < vTo)
                return AccDecDirection.Acc;
            return AccDecDirection.Dec;
        }

        public static double GetDirectedJerk(double jMax, double vFrom, double vTo)
        {
            var direction = GetAccDecDirection(vFrom, vTo);
            return GetDirectedJerk(jMax, direction);
        }

        public static double GetDirectedJerk(double jMax, AccDecDirection direction)
        {
            switch (direction)
            {
                case AccDecDirection.Constant:
                case AccDecDirection.Acc:
                    return jMax;
                case AccDecDirection.Dec:
                    return -jMax;
                default:
                    return double.NaN;
            }
        }

        #region CalculateTimes

        internal (double d1, double d2) CalculateTimes(double distance, double vTarget = double.NaN)
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
            var path = 1;
            var dForAMax = Math.Abs(_aMax / _jMax);
            var d1 = dForAMax;
            var d2 = 0.0;
            CalculateEndValues(d1, d2, _jMax, out var v3, out var s3);

            if (s3 < distance)
            {
                // The previously calculated profile does not even cover the given distance
                // => Phase 2 with constant acceleration is necessary
                // => d2 > 0
                double d2_1, d2_2;
                (d1, d2_1, d2_2) = CalculateTimesWithConstantA(distance, dForAMax);

                d2 = SelectPlausibleD2(distance, vTarget, d1, d2_1, d2_2);
                path = 2;
            }
            else if (s3 > distance)
            {
                // The previously calculated profile overshoots the given distance.
                // To stay within the given distance, aMax obviously must not be reached.
                // => d1 < dForAMax!!
                (d1, d2) = CalculateTimesWithANotReached(distance);
                path = 3;
            }

            CalculateEndValues(d1, d2, _jMax, out v3, out s3);

            if (Math.Abs(s3 - distance) > 1e-8)
            {
                throw new InvalidOperationException($"Invalid calculated distance {s3:N3} on path {path}, target would be {distance:N3}");
            }

            return (d1, d2);
        }

        private double SelectPlausibleD2(double distance, double vTarget, double d1, double d2_1, double d2_2)
        {
            double d2;
            if (d2_1 < 0)
            {
                // negative duration is not an option
                d2 = d2_2;
            }
            else if (d2_2 < 0)
            {
                // negative duration is not an option
                d2 = d2_1;
            }
            else
            {
                CalculateEndValues(d1, d2_1, _jMax, out var v3_1, out var s3_1);
                CalculateEndValues(d1, d2_2, _jMax, out var v3_2, out var s3_2);

                if (!double.IsNaN(vTarget))
                {
                    // vTarget is set, take solution where vTarget = v3
                    if (Math.Abs(v3_1 - vTarget) < 1e-8)
                    {
                        d2 = d2_1;
                    }
                    else if (Math.Abs(v3_2 - vTarget) < 1e-8)
                    {
                        d2 = d2_2;
                    }
                    else
                    {
                        throw new InvalidOperationException($"vTarget set ({vTarget:N3}mm/s) but not reached");
                    }
                }
                else if (Math.Abs(s3_1 - distance) > 1e-8 || v3_1 < 0)
                {
                    d2 = d2_2;
                }
                else
                {
                    d2 = d2_1;
                }
            }

            return d2;
        }

        private (double d1, double d2) CalculateTimesWithANotReached(double distance)
        {
            var d1_Mats = AnsatzMatz(distance, _jMax);

#pragma warning disable S1481 // variables used for debugging
            var d1_Wolfram = AnsatzWolframAlpha(distance);

            var solverResult = SolveCubic(Math.Abs(_jMax), 0, 2 * _v0, -distance);
#pragma warning restore S1481

            var d1 = d1_Mats;

            if (double.IsNaN(d1) || double.IsInfinity(d1) || d1 < 0)
            {
                throw new InvalidOperationException($"d1 invalid ({d1})");
            }

            return (d1, 0);

        }

        //TODO check if valid, then refurbish or remove
        private double AnsatzWolframAlpha(double distance)
        {
            var jAbs = _jMax;
            var j_2 = jAbs * jAbs;
            var j_3 = j_2 * jAbs;
            var j_4 = j_3 * jAbs;

            // Wolfram Alphas output splitted in single parts for better readability
            // https://www.wolframalpha.com/input/?i=2vx%2Bjx%5E3-s%3D0
            var p1 = Math.Sqrt(27 * j_4 * distance * distance + 32 * j_3 * _v0 * _v0 * _v0);
            var p2 = CubeRoot.Get(9 * j_2 * distance + Math.Sqrt(3) * p1);
            var p3 = CubeRoot.Get(2) * Math.Pow(3, 2.0 / 3) * jAbs;
            var p4 = 2 * CubeRoot.Get(2.0 / 3) * _v0;
            var p5 = CubeRoot.Get(9 * j_2 * distance + Math.Sqrt(3) * p1);

            return p2 / p3 - p4 / p5;
        }

        //TODO check if valid, then refurbish or remove
        private double AnsatzMatz(double distance, double jAbs)
        {
            var A = jAbs;
            var B = 0;
            var C = 2 * _v0;
            var D = -distance;
            var p = (9 * A * C - 3 * B * B) / (9 * A * A);
            var q = (2 * B * B * B - 9 * A * B * C + 27 * A * A * D) / (27 * A * A * A);
            var diskriminante = (27 * A * A * D * D + 4 * B * B * B * D - 18 * A * B * C * D + 4 * A * C * C * C - B * B * C * C) / (108 * A * A * A * A);
            var d1Mats = double.NaN;
            if (true) // p < 0
            {
                var h1 = Math.Sqrt(-p * 4 / 3);
                var h2 = -q / 2 * Math.Sqrt(-27.0 / (p * p * p));
                if (Math.Abs(h2) > 1)
                {
                    // ERROR!
                }
                var h3 = (1.0 / 3) * Math.Acos(h2);
                var h4 = B / (3 * A);
                if (diskriminante < 0)
                {
                    var x2 = -h1 * Math.Cos(h3 + (Math.PI / 3)) - h4;
                    var x1 = h1 * Math.Cos(h3) - h4;
                    var x3 = -h1 * Math.Cos(h3 - Math.PI / 3) - h4;

                    if (Math.Abs(x1) < Math.Abs(x2) && Math.Abs(x1) < Math.Abs(x3))
                    {
                        d1Mats = x1;
                    }
                    else if (Math.Abs(x2) < Math.Abs(x1) && Math.Abs(x2) < Math.Abs(x3))
                    {
                        d1Mats = x2;
                    }
                    else
                    {
                        d1Mats = x3;
                    }
                }
                else if (diskriminante == 0 && p == 0)
                {
                    d1Mats = -B / (3 * A);
                }
                else if (diskriminante == 0 && p != 0)
                {
                    var x1 = (B * B * B - 4 * A * B * C + 9 * A * A * D) / (3 * A * A * C - A * B * B);
                    var x2 = (A * B * C - 9 * A * A * D) / (6 * A * A * C - 2 * A * B * B);
                    if (Math.Abs(x1) < Math.Abs(x2))
                    {
                        d1Mats = x1;
                    }
                    else
                    {
                        d1Mats = x2;
                    }
                }
                else if (diskriminante > 0)
                {
                    var u = -(q / 2) + Math.Sqrt(diskriminante);
                    if (u >= 0)
                    {
                        u = CubeRoot.Get(u);
                    }
                    else
                    {
                        u = -CubeRoot.Get(-u);
                    }
                    var v = -(q / 2) - Math.Sqrt(diskriminante);
                    if (v >= 0)
                    {
                        v = CubeRoot.Get(v);
                    }
                    else
                    {
                        v = -CubeRoot.Get(-v);
                    }

                    d1Mats = u + v - (B / (3 * A));
                }
            }
            else
            {
                // FUCK
            }

            return d1Mats;
        }

        //TODO check if valid, then refurbish or remove
        public List<Complex> SolveCubic(double a, double b, double c, double d)
        {
            const int NRoots = 3;
            var SquareRootof3 = Math.Sqrt(3);
            // the 3 cubic roots of 1
            var CubicUnity = new List<Complex>(NRoots)
                        { new Complex(1, 0), new Complex(-0.5, -SquareRootof3 / 2.0), new Complex(-0.5, SquareRootof3 / 2.0) };
            // intermediate calculations
            var DELTA = 18 * a * b * c * d - 4 * b * b * b * d + b * b * c * c - 4 * a * c * c * c - 27 * a * a * d * d;
            var DELTA0 = b * b - 3 * a * c;
            var DELTA1 = 2 * b * b * b - 9 * a * b * c + 27 * a * a * d;
            Complex DELTA2 = -27 * a * a * DELTA;
            var C = Complex.Pow((DELTA1 + Complex.Pow(DELTA2, 0.5)) / 2, 1 / 3.0); //Phew...
            var R = new List<Complex>(NRoots);
            for (var i = 0; i < NRoots; i++)
            {
                var M = CubicUnity[i] * C;
                var Root = -1.0 / (3 * a) * (b + M + DELTA0 / M);
                R.Add(Root);
            }
            return R;
        }

        private (double d1, double d2_1, double d2_2) CalculateTimesWithConstantA(double distance, double d1)
        {
            var jAbs = Math.Abs(_jMax);
            // solving quadratic equation using "Mitternachtsformel"
            var mnfA = 0.5 * jAbs * d1;
            var mnfB = _v0 + 1.5 * jAbs * d1 * d1;
            var mnfC = 2 * _v0 * d1 + jAbs * d1 * d1 * d1 - distance;

            // "Mitternachtsformel" provides two results
            var d2_1 = (-mnfB + Math.Sqrt(mnfB * mnfB - 4 * mnfA * mnfC)) / (2 * mnfA);
            var d2_2 = (-mnfB - Math.Sqrt(mnfB * mnfB - 4 * mnfA * mnfC)) / (2 * mnfA);

            return (d1, d2_1, d2_2);
        }

        #endregion

        internal void CalculateEndValues(double d1, double d2, double jMaxDirected, out double v3, out double s3)
        {
            GetStatus(d1 + d2 + d1, _v0, d1, d2, jMaxDirected, out _, out _, out v3, out s3);
        }

        #region Calculations from P2PCalculator, extended by v0

        public static void GetStatus(double t, double v0, double d1, double d2, double jMaxDirected, out double j, out double a, out double v, out double s)
        {
            var t1 = d1;
            var t2 = t1 + d2;
            var t3 = t2 + d1;

            if (t <= t1)
            {
                GetStatus1(t, v0, jMaxDirected, out j, out a, out v, out s);
            }
            else
            {
                GetStatus1(t1, v0, jMaxDirected, out var j1, out var a1, out var v1, out var s1);

                if (t <= t2)
                {
                    GetStatus2(t, t1, a1, v1, s1, out j, out a, out v, out s);
                }
                else
                {
                    GetStatus2(t2, t1, a1, v1, s1, out var j2, out var a2, out var v2, out var s2);

                    if (t <= t3)
                    {
                        GetStatus3(t, t2, a2, v2, s2, jMaxDirected, out j, out a, out v, out s);
                    }
                    else
                    {
                        throw new ArgumentOutOfRangeException($"Given time t must be lower than t3 which is {t3:N2} at the moment");
                    }
                }
            }
        }

        private static void GetStatus1(double t, double v0, double jMaxDirected, out double j, out double a, out double v, out double s)
        {
            j = jMaxDirected;
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

        private static void GetStatus3(double t, double t2, double a2, double v2, double s2, double jMaxDirected, out double j, out double a, out double v, out double s)
        {
            var tPhase = t - t2;
            var tPhase2 = tPhase * tPhase;
            var tPhase3 = tPhase2 * tPhase;
            j = -jMaxDirected;
            a = a2 - jMaxDirected * tPhase;
            v = v2 + a2 * tPhase + 0.5 * -jMaxDirected * tPhase2;
            s = s2 + v2 * tPhase + 0.5 * a2 * tPhase2 + -jMaxDirected / 6 * tPhase3;
        }

        #endregion
    }
}
