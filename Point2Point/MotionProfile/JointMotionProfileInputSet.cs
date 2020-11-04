using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Point2Point.Mathematics.ExtendedP2P;

namespace Point2Point.JointMotion
{
    public class JointMotionProfileInputSet
    {
        public MotionParameter Parameters { get; }
        public double InitialAcceleration { get; }
        public double InitialVelocity { get; }
        public ConstraintsCollection Constraints { get; }

        public JointMotionProfileInputSet(MotionParameter parameters, double initialAcceleration, double initialVelocity, ConstraintsCollection constraints)
        {
            Parameters = parameters;
            InitialAcceleration = initialAcceleration;
            InitialVelocity = initialVelocity;
            Constraints = constraints;
        }
    }
}
