using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point
{

    public static class Vector3Extensions
    {
        public static bool IsCollinearWith(this Vector3 a, Vector3 b)
        {
            var squaredLength = Vector3.Cross(a, b).LengthSquared();
            return squaredLength < 1e-8;
        }

        public static bool IsVertical(this Vector3 v)
            => v.Z != 0 && v.X == 0 && v.Y == 0;

        public static bool IsUp(this Vector3 v)
            => v.Z > 0 && v.X == 0 && v.Y == 0;

        public static bool IsDown(this Vector3 v)
            => v.Z < 0 && v.X == 0 && v.Y == 0;

        public static bool HasDirectionY(this Vector3 v)
            => v.Y != 0 && v.X == 0 && v.Z == 0;

        public static bool HasDirectionX(this Vector3 v)
            => v.X != 0 && v.Y == 0 && v.Z == 0;

        public static bool HasSameDirection(this Vector3 a, Vector3 b)
            => (Vector3.Normalize(a) - Vector3.Normalize(b)).Length() < 1e-3;

        public static double AngleBetween(this Vector3 vector1, Vector3 vector2)
        {
            var v1Normalized = Vector3.Normalize(vector1);
            var v2Normalized = Vector3.Normalize(vector2);

            var ratio = Vector3.Dot(v1Normalized, v2Normalized);

            // The "straight forward" method of acos(u.v) has large precision
            // issues when the dot product is near +/-1.  This is due to the 
            // steep slope of the acos function as we approach +/- 1.  Slight 
            // precision errors in the dot product calculation cause large
            // variation in the output value. 
            //
            //        |                   |
            //         \__                |
            //            ---___          | 
            //                  ---___    |
            //                        ---_|_ 
            //                            | ---___ 
            //                            |       ---___
            //                            |             ---__ 
            //                            |                  \
            //                            |                   |
            //       -|-------------------+-------------------|-
            //       -1                   0                   1 
            //
            //                         acos(x) 
            // 
            // To avoid this we use an alternative method which finds the
            // angle bisector by (u-v)/2: 
            //
            //                            _>
            //                       u  _-  \ (u-v)/2
            //                        _-  __-v 
            //                      _=__--
            //                    .=-----------> 
            //                            v 
            //
            // Because u and v and unit vectors, (u-v)/2 forms a right angle 
            // with the angle bisector.  The hypotenuse is 1, therefore
            // 2*asin(|u-v|/2) gives us the angle between u and v.
            //
            // The largest possible value of |u-v| occurs with perpendicular 
            // vectors and is sqrt(2)/2 which is well away from extreme slope
            // at +/-1. 
            // 
            // (See Windows OS Bug #1706299 for details)

            double theta;

            if (ratio < 0)
            {
                theta = Math.PI - 2.0 * Math.Asin((-v1Normalized - v2Normalized).Length() / 2.0);
            }
            else
            {
                theta = 2.0 * Math.Asin((v1Normalized - v2Normalized).Length() / 2.0);
            }

            return theta / Math.PI * 180.0;
        }

        public static Vector3 Abs(this Vector3 vector)
            => new Vector3(Math.Abs(vector.X), Math.Abs(vector.Y), Math.Abs(vector.Z));

        /// <summary>
        /// Method converts an angle to heading vector
        /// </summary>
        /// <param name="angle">Input angle in rad [-Pi to Pi]</param>
        /// <returns>heading vector</returns>
        public static Vector3 GetVector3FromAngle(double angle)
        {
            return new Vector3((float) Math.Cos(angle), (float) Math.Sin(angle), 0);
        }
    }
}
