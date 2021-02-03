using Xunit;
using Point2Point.JointMotion;
using Point2Point.Mathematics.ExtendedP2P;

namespace Point2Point.Tests
{
    
    public class JointMotionProfileTests
    {
        private MotionParameter _parameters = new MotionParameter(2000, -2000, 1000, -1000);

        [Fact]
        public void StartAndEndStatusAreCorrect()
        {
            var jmp = new JointMotionProfile(_parameters,
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 200),
                new VelocityConstraint(2000, 1000, 800));

            jmp.GetStatus(0, out _, out var v0, out var s0);

            Assert.Equal(0.0, v0);
            Assert.Equal(0.0, s0);
            Assert.NotEqual(0.0, jmp.TotalDuration);

            jmp.GetStatus(jmp.TotalDuration, out _, out var vEnd, out var sEnd);

            Assert.Equal(0.0, vEnd);
            Assert.Equal(3000.0, sEnd);
        }
    }
}
