using System;

namespace Point2Point.JointMotion
{
    public class VelocityConstraint : IComparable
    {
        /// <summary>
        /// Distance [mm] at which the constraint starts
        /// </summary>
        public double Start { get; set; }

        /// <summary>
        /// Length [mm] of the constraint
        /// </summary>
        public double Length { get; set; }

        /// <summary>
        /// Maximum allowed velocity [mm/s] within this constraint
        /// </summary>
        public double MaximumVelocity { get; set; }

        /// <summary>
        /// Distance [mm] at which the constraint ends
        /// </summary>
        public double End => Start + Length;

        public VelocityConstraint(double start, double length, double maximumVelocity)
        {
            Start = start;
            Length = length;
            MaximumVelocity = maximumVelocity;
        }

        /// <summary>
        /// Checks wether the given distance [mm] lies within this constraint.
        /// </summary>
        /// <param name="distance">Distance to check</param>
        /// <returns>True if the given distance is within the constraint</returns>
        public bool Contains(double distance) 
            => Start <= distance && End > distance;

        /// <summary>
        /// Creates a copy of this constraint with no references to this constraint
        /// </summary>
        /// <returns>A complete copy of this object</returns>
        public VelocityConstraint Copy() 
            => new VelocityConstraint(Start, Length, MaximumVelocity);

        /// <summary>
        /// Compares two VelocityConstraints by their MaximumVelocity
        /// </summary>
        /// <param name="obj">Object to compare to</param>
        /// <returns>An integer indicating the comparing</returns>
        public int CompareTo(object obj)
        {
            if (obj is VelocityConstraint constraint)
            {
                return MaximumVelocity.CompareTo(constraint.MaximumVelocity);
            }

            return 0;
        }

        public static bool operator >(VelocityConstraint a, VelocityConstraint b)
            => a.MaximumVelocity > b.MaximumVelocity;

        public static bool operator <(VelocityConstraint a, VelocityConstraint b)
            => a.MaximumVelocity < b.MaximumVelocity;
    }
}
