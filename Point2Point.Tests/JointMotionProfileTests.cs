using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Point2Point.JointMotion;
using Point2Point.Mathematics.ExtendedP2P;

namespace Point2Point.Tests
{
    [TestClass]
    public class JointMotionProfileTests
    {
        private MotionParameter _parameters = new MotionParameter(2000, -2000, 1000, -1000);

        [TestMethod]
        public void StartAndEndStatusAreCorrect()
        {
            // var jmp = new JointMotionProfile(_parameters,
            //     new VelocityConstraint(0, 1000, 500),
            //     new VelocityConstraint(1000, 1000, 200),
            //     new VelocityConstraint(2000, 1000, 800));
            //
            // jmp.GetStatus(0, out _, out var v0, out var s0);
            //
            // Assert.AreEqual(0.0, v0);
            // Assert.AreEqual(0.0, s0);
            // Assert.AreNotEqual(0.0, jmp.TotalDuration);
            //
            // jmp.GetStatus(jmp.TotalDuration, out _, out var vEnd, out var sEnd);
            //
            // Assert.AreEqual(0.0, vEnd);
            // Assert.AreEqual(3000.0, sEnd);
        }


        [TestMethod]
        public void RotationPointProfile()
        {
            var velocityConstraintsCollection = new VelocityConstraintsCollection()
            {
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(500, 0, 0),
                
                // new VelocityConstraint(1000, 1000, 200),
                // new VelocityConstraint(2000, 1000, 800)
            };

            var stopConstraintsCollection = new StopConstraintCollection()
            {
                new StopConstraint(500, TimeSpan.FromSeconds(5))
            };


            var jmp = new JointMotionProfile(_parameters, velocityConstraintsCollection, stopConstraintsCollection);

            jmp.GetStatus(0, out _, out var v0, out var s0);

            Assert.AreEqual(0.0, v0);
            Assert.AreEqual(0.0, s0);
            Assert.AreNotEqual(0.0, jmp.TotalDuration);

            jmp.GetStatus(jmp.TotalDuration, out _, out var vEnd, out var sEnd);

            Assert.AreEqual(0.0, vEnd);
            Assert.AreEqual(3000.0, sEnd);
        }
    }
}