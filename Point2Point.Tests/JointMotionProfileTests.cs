using Microsoft.VisualStudio.TestTools.UnitTesting;
using Point2Point.JointMotion;

namespace Point2Point.Tests
{
    [TestClass]
    public class JointMotionProfileTests
    {
        [TestMethod]
        public void MyTestMethod()
        {
            var jMax = 2000;
            var aMax = 1000;

            double reachable;
            reachable = P2PRamp.GetReachableVelocity(250, 0, jMax, aMax);
            reachable = P2PRamp.GetReachableVelocity(500, 0, jMax, aMax);
            reachable = P2PRamp.GetReachableVelocity(100, 0, jMax, aMax);
            reachable = P2PRamp.GetReachableVelocity(100, 100, jMax, aMax);

            var constraints = new ConstraintsCollection(
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(500, 1000, 200));

            var parameters = new P2PParameters(2000, 500, 1000);

            //var jointMotion = JointMotionProfile.CalculateProfile(constraints, parameters);
        }
    }
}
