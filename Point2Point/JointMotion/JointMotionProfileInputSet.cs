using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Shuttles.Base.Devices.Shuttles.Motion.Ramp;

namespace Point2Point.JointMotion
{
    public class JointMotionProfileInputSet
    {
        public RampMotionParameter Parameters { get; }
        public double InitialAcceleration { get; }
        public double InitialVelocity { get; }
        public ConstraintsCollection Constraints { get; }

        public JointMotionProfileInputSet(RampMotionParameter parameters, double initialAcceleration, double initialVelocity, ConstraintsCollection constraints)
        {
            Parameters = parameters;
            InitialAcceleration = initialAcceleration;
            InitialVelocity = initialVelocity;
            Constraints = constraints;
        }
    }
}
