using System;

namespace Point2Point.JointMotion
{
    public class VelocityConstraint
    {
        private readonly string _caller;

        /// <summary>
        /// Distance [mm] at which the constraint starts
        /// </summary>
        public double Start { get; private set; } 

        /// <summary>
        /// Length [mm] of the constraint
        /// </summary>
        public double Length { get; private set; }

        /// <summary>
        /// Maximum allowed velocity [mm/s] within this constraint
        /// </summary>
        public double MaximumVelocity { get; private set; }

        /// <summary>
        /// Distance [mm] at which the constraint ends
        /// </summary>
        public double End => Start + Length;

        public VelocityConstraint(double start, double length, double maximumVelocity, string caller)
        {
            _caller = caller;
            
            Start = start;
            Length = length;
            MaximumVelocity = maximumVelocity;
            CheckConstraint(caller);
        }

        public void SetStart(double start, string caller)
        {
            Start = start; 
            CheckConstraint(caller);
        }
        public void SetLength(double length, string caller)
        {
            Length = length; 
            CheckConstraint(caller);
        }
        
        public void SetMaximumVelocity(double maximumVelo, string caller)
        {
            MaximumVelocity = maximumVelo; 
            CheckConstraint(caller);
        }

        public void CheckConstraint(string caller)
        {
            if (Length < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(Length), $"Length must be greater than zero");
            }
            if (MaximumVelocity < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumVelocity), $"MaximumVelocity must be greater than zero");
            }
            
            if (Length > 0 && MaximumVelocity == 0)
            {
                throw new ArgumentOutOfRangeException(nameof(MaximumVelocity), $"Constraint with length > 0 must be greater than zero");
            }
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
        public VelocityConstraint Copy(string caller)
            => new VelocityConstraint(Start, Length, MaximumVelocity, caller );

        /// <summary>
        /// Reduces the MaximumVelocity of this constraint by the given velocity.
        /// </summary>
        /// <param name="velocity">Velocity [mm/s] by which the given velocity shoudl be reduced</param>
        public void ReduceBy(double velocity, string caller)
        {
            MaximumVelocity -= velocity;
        }

        #region Operators between to constraints

        public static bool operator >(VelocityConstraint a, VelocityConstraint b)
            => a.MaximumVelocity > b.MaximumVelocity;

        public static bool operator <(VelocityConstraint a, VelocityConstraint b)
            => a.MaximumVelocity < b.MaximumVelocity;

        public static bool operator >=(VelocityConstraint a, VelocityConstraint b)
            => a.MaximumVelocity >= b.MaximumVelocity;

        public static bool operator <=(VelocityConstraint a, VelocityConstraint b)
            => a.MaximumVelocity <= b.MaximumVelocity;

        public static double operator -(VelocityConstraint a, VelocityConstraint b)
            => a.MaximumVelocity - b.MaximumVelocity;

        public static double operator +(VelocityConstraint a, VelocityConstraint b)
            => a.MaximumVelocity - b.MaximumVelocity;

        #endregion

        #region Operators between constraint and double

        public static bool operator >(double a, VelocityConstraint b)
            => a > b.MaximumVelocity;

        public static bool operator <(double a, VelocityConstraint b)
            => a < b.MaximumVelocity;

        public static bool operator >(VelocityConstraint a, double b)
            => a.MaximumVelocity > b;

        public static bool operator <(VelocityConstraint a, double b)
            => a.MaximumVelocity < b;

        public static double operator -(VelocityConstraint a, double b)
            => a.MaximumVelocity - b;

        public static double operator +(VelocityConstraint a, double b)
            => a.MaximumVelocity + b;

        #endregion
    }
}
