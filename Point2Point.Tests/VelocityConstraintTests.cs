using Microsoft.VisualStudio.TestTools.UnitTesting;
using Point2Point.JointMotion;

namespace Point2Point.Tests
{
    [TestClass]
    public class VelocityConstraintTests
    {
        [TestMethod]
        public void RelationalOperatorsWork()
        {
            // Arrange
            var con0 = new VelocityConstraint(0, 0, 100);
            var con1 = new VelocityConstraint(0, 0, 200);
            var con2 = new VelocityConstraint(0, 0, 200);

            // Act & Assert
            Assert.IsTrue(con0 < con1);
            Assert.IsFalse(con0 > con1);
            Assert.IsFalse(con1 > con2);
            Assert.IsFalse(con1 < con2);
        }
    }
}
