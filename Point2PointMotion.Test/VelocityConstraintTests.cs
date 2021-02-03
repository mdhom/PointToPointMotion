using System;
using Point2Point.JointMotion;
using Xunit;

namespace Point2Point.Tests
{
    public class VelocityConstraintTests
    {
        [Fact]
        public void ParametersAreValidated()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => new VelocityConstraint(0, -1, 100));
            Assert.Throws<ArgumentOutOfRangeException>(() => new VelocityConstraint(0, 0, 100));
            Assert.Throws<ArgumentOutOfRangeException>(() => new VelocityConstraint(0, 100, -1));
            Assert.Throws<ArgumentOutOfRangeException>(() => new VelocityConstraint(0, 100, 0));
        }

        [Fact]
        public void RelationalOperatorsWork()
        {
            // Arrange
            var con0 = new VelocityConstraint(0, 10, 100);
            var con1 = new VelocityConstraint(0, 10, 200);
            var con2 = new VelocityConstraint(0, 10, 200);

            // Act & Assert
            Assert.True(con0 < con1);
            Assert.False(con1 < con2);

            Assert.False(con1 <= con0);
            Assert.True(con1 <= con2);

            Assert.False(con0 > con1);
            Assert.False(con1 > con2);

            Assert.False(con0 >= con1);
            Assert.True(con1 >= con2);

            Assert.True(con0 < 150.0);
            Assert.True(con0 > 50.0);
            Assert.True(150.0 > con0);
            Assert.True(50.0  < con0);
        }

        [Fact]
        public void MathematicalOperatorsWork()
        {
            // Arrange
            var con0 = new VelocityConstraint(0, 10, 100);
            var con1 = new VelocityConstraint(0, 10, 200);
            var con2 = new VelocityConstraint(0, 10, 200);

            // Act
            var diff0 = con1 - con0;
            var diff1 = con2 - con1;
            var diff2 = con0 - con1;
            var diff3 = con0 - 50.0;
            var sum0 = con0 + 100.0;

            // Assert
            Assert.Equal( 100.0, diff0);
            Assert.Equal(   0.0, diff1);
            Assert.Equal(-100.0, diff2);
            Assert.Equal(  50.0, diff3);
            Assert.Equal( 200.0, sum0);
        }

        [Fact]
        public void ReduceByWorks()
        {
            // Arrange
            var con = new VelocityConstraint(0, 10, 100);

            // Act
            con.ReduceBy(50);

            // Assert
            Assert.Equal(50.0, con.MaximumVelocity);
        }

        [Fact]
        public void ContainsWorks()
        {
            // Arrange
            var con = new VelocityConstraint(0, 100, 100);

            // Act
            var contains0 = con.Contains(0);
            var contains1 = con.Contains(50);
            var contains2 = con.Contains(100);

            // Assert
            Assert.True(contains0);
            Assert.True(contains1);
            Assert.False(contains2);
        }

        [Fact]
        public void CopyReturnsDeepCopy()
        {
            // Assert
            var con = new VelocityConstraint(0, 50, 100);

            // Act
            var conCopy = con.Copy();

            // Assert that copy has same values as original
            Assert.Equal(con.Start, conCopy.Start);
            Assert.Equal(con.Length, conCopy.Length);
            Assert.Equal(con.End, conCopy.End);
            Assert.Equal(con.MaximumVelocity, conCopy.MaximumVelocity);

            // Check that no references to original exist anymore
            conCopy.MaximumVelocity /= 2;

            Assert.Equal(100, con.MaximumVelocity);
            Assert.Equal(50, conCopy.MaximumVelocity);
        }
    }
}