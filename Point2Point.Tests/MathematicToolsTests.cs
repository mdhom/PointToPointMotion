using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Point2Point.Tests
{
    [TestClass]
    public class MathematicToolsTests
    {
        [TestMethod]
        public void CubeRootWorking()
        {
            var random = new Random((int)DateTime.Now.Ticks);
            for (var i = 0; i < 10; i++)
            {
                var number = random.NextDouble() * 1e6;
                var root = MathematicTools.GetCubeRoot(Math.Pow(number, 3));

                Assert.AreEqual(number, root, 1e-5);
            }
        }

        [TestMethod]
        [DataRow(0.0, 1.0, 2.0, true, -2.0, -2.0, DisplayName = "Linear equation")]
        [DataRow(1.0, 5.0, 2.0, true, -0.43844, -4.56155, DisplayName = "Quadratic equation")]
        [DataRow(5.0, 1.0, 1.0, false, 0, 0, DisplayName = "Quadratic equation with negative determinant")]
        public void SolveEquationSolvesLinearEquation(double a, double b, double c, bool expectedSuccess, double expectedX1, double expectedX2)
        {
            var success = MathematicTools.SolveEquation(a, b, c, out var x1, out var x2);

            Assert.AreEqual(expectedSuccess, success);
            Assert.AreEqual(expectedX1, x1, 1e-5);
            Assert.AreEqual(expectedX2, x2, 1e-5);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentOutOfRangeException))]
        public void SolveEquationValidatesInput()
        {
            MathematicTools.SolveEquation(0, 0, 0, out _, out _);
            Assert.Fail();
        }
    }
}
