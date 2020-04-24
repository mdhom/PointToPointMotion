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
            Assert.IsFalse(con1 < con2);

            Assert.IsFalse(con1 <= con0);
            Assert.IsTrue(con1 <= con2);

            Assert.IsFalse(con0 > con1);
            Assert.IsFalse(con1 > con2);

            Assert.IsFalse(con0 >= con1);
            Assert.IsTrue(con1 >= con2);

            Assert.IsTrue(con0 < 150.0);
            Assert.IsTrue(con0 > 50.0);
            Assert.IsTrue(150.0 > con0);
            Assert.IsTrue(50.0  < con0);
        }

        [TestMethod]
        public void MathematicalOperatorsWork()
        {
            // Arrange
            var con0 = new VelocityConstraint(0, 0, 100);
            var con1 = new VelocityConstraint(0, 0, 200);
            var con2 = new VelocityConstraint(0, 0, 200);

            // Act
            var diff0 = con1 - con0;
            var diff1 = con2 - con1;
            var diff2 = con0 - con1;
            var diff3 = con0 - 50.0;
            var sum0 = con0 + 100.0;

            // Assert
            Assert.AreEqual( 100.0, diff0);
            Assert.AreEqual(   0.0, diff1);
            Assert.AreEqual(-100.0, diff2);
            Assert.AreEqual(  50.0, diff3);
            Assert.AreEqual( 200.0, sum0);
        }

        [TestMethod]
        public void ReduceByWorks()
        {
            // Arrange
            var con = new VelocityConstraint(0, 0, 100);

            // Act
            con.ReduceBy(50);

            // Assert
            Assert.AreEqual(50.0, con.MaximumVelocity);
        }

        [TestMethod]
        public void ContainsWorks()
        {
            // Arrange
            var con = new VelocityConstraint(0, 100, 100);

            // Act
            var contains0 = con.Contains(0);
            var contains1 = con.Contains(50);
            var contains2 = con.Contains(100);

            // Assert
            Assert.IsTrue(contains0);
            Assert.IsTrue(contains1);
            Assert.IsFalse(contains2);
        }

        [TestMethod]
        public void CopyReturnsDeepCopy()
        {
            // Assert
            var con = new VelocityConstraint(0, 50, 100);

            // Act
            var conCopy = con.Copy();

            // Assert that copy has same values as original
            Assert.AreEqual(con.Start, conCopy.Start);
            Assert.AreEqual(con.Length, conCopy.Length);
            Assert.AreEqual(con.End, conCopy.End);
            Assert.AreEqual(con.MaximumVelocity, conCopy.MaximumVelocity);

            // Check that no references to original exist anymore
            conCopy.MaximumVelocity /= 2;

            Assert.AreEqual(100, con.MaximumVelocity);
            Assert.AreEqual(50, conCopy.MaximumVelocity);
        }
    }
}