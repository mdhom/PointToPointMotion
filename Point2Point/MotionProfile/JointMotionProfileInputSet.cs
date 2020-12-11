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
        public VelocityConstraintsCollection VelocityConstraints { get; }

        public StopConstraintCollection StopConstraints { get; set; }

        public JointMotionProfileInputSet(MotionParameter parameters, double initialAcceleration,
            double initialVelocity, VelocityConstraintsCollection velocityConstraints,
            StopConstraintCollection stopConstraints)
        {
            Parameters = parameters;
            InitialAcceleration = initialAcceleration;
            InitialVelocity = initialVelocity;
            VelocityConstraints = velocityConstraints;
            StopConstraints = stopConstraints;
        }
    }
}