using System;

namespace Point2Point.JointMotion
{
    public class StopConstraint
    {
        /// <summary>
        /// Distance [mm] at which the constraint starts
        /// </summary>
        public double Start { get; set; }
        
        /// <summary>
        /// Time [s] motion is stopped
        /// </summary>
        public TimeSpan TimeSpan { get; set; }


        public StopConstraint(double start, TimeSpan timeSpan)
        {
            Start = start;
            TimeSpan = timeSpan;
        }

      

        /// <summary>
        /// Creates a copy of this constraint with no references to this constraint
        /// </summary>
        /// <returns>A complete copy of this object</returns>
        public StopConstraint Copy()
            => new StopConstraint(Start, TimeSpan);
        

        #region Operators between to constraints


        #endregion
    }
}