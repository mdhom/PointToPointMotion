using Microsoft.VisualStudio.TestTools.UnitTesting;
using Point2Point.JointMotion;
using Shuttles.Base.Devices.Shuttles.Motion.Ramp;

namespace Point2Point.Tests
{
    [TestClass]
    public class JointMotionProfileTests
    {
        private RampMotionParameter _parameters = new RampMotionParameter(2000, -2000, 1000, -1000);

        [TestMethod]
        public void StartAndEndStatusAreCorrect()
        {
            var jmp = new JointMotionProfile(_parameters,
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 200),
                new VelocityConstraint(2000, 1000, 800));

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
