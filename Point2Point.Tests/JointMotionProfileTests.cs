using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
            var constraints = new ConstraintsCollection(
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(500, 1000, 200));

            var parameters = new P2PParameters(2000, 500, 1000);

            var jointMotion = new JointMotionProfile(constraints, parameters);
        }
    }
}
