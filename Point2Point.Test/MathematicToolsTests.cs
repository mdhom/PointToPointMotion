using System;
using Xunit;

namespace Point2Point.Tests
{
    public class MathematicToolsTests
    {
        [Fact]
        public void CubeRootWorking()
        {
            var random = new Random((int)DateTime.Now.Ticks);
            for (var i = 0; i < 10; i++)
            {
                var number = random.NextDouble() * 1e6;
                var root = MathematicTools.GetCubeRoot(Math.Pow(number, 3));

                Assert.Equal(number, root, 5);
            }
        }

        [Theory]
        [InlineData(0.0, 1.0, 2.0, true, -2.0, -2.0)] //Linear equation
        [InlineData(1.0, 5.0, 2.0, true, -0.43844, -4.56155)] //Quadratic equation
        [InlineData(5.0, 1.0, 1.0, false, 0, 0)] //Quadratic equation with negative determinant
        public void SolveEquationSolvesLinearEquation(double a, double b, double c, bool expectedSuccess, double expectedX1, double expectedX2)
        {
            var success = MathematicTools.SolveEquation(a, b, c, out var x1, out var x2);

            Assert.Equal(expectedSuccess, success);
            Assert.Equal(expectedX1, x1, 4);
            Assert.Equal(expectedX2, x2, 4);
        }

        [Fact]
        public void SolveEquationValidatesInput()
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => MathematicTools.SolveEquation(0, 0, 0, out _, out _));
        }
    }
}
