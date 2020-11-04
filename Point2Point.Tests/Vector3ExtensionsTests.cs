using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point.Tests
{
    [TestClass]
    public class Vector3ExtensionsTests
    {
        [TestMethod]
        [DynamicData(nameof(GetTestDataAngleBetweenWorks), DynamicDataSourceType.Method)]
        public void AngleBetweenWorks(Vector3 v0, Vector3 v1, double expectedAngle)
        {
            var angle = v0.AngleBetween(v1);

            Assert.AreEqual(expectedAngle, angle, 0.001);
        }
        public static IEnumerable<object[]> GetTestDataAngleBetweenWorks()
        {
            yield return new object[] { Vector3.UnitX, Vector3.UnitY, 90.0 };
            yield return new object[] { Vector3.UnitX, Vector3.UnitZ, 90.0 };
            yield return new object[] { Vector3.UnitX, Vector3.UnitX, 0.0 };
            yield return new object[] { Vector3.UnitX, new Vector3(1, 1, 0), 45.0 };
        }

        [TestMethod]
        public void AbsWorks()
        {
            // Arrange
            var v = new Vector3(-1, -2, -3);

            // Act
            var absV = v.Abs();

            // Assert
            Assert.AreEqual(1f, absV.X);
            Assert.AreEqual(2f, absV.Y);
            Assert.AreEqual(3f, absV.Z);
        }

        [TestMethod]
        [DynamicData(nameof(GetTestDataHasSameDirectionWorks), DynamicDataSourceType.Method)]
        public void HasSameDirectionWorks(Vector3 vector1, Vector3 vector2, bool expectedResult)
        {
            var result = vector1.HasSameDirection(vector2);
            Assert.AreEqual(expectedResult, result);
        }

        public static IEnumerable<object[]> GetTestDataHasSameDirectionWorks()
        {
            yield return new object[] { Vector3.UnitX, Vector3.UnitX * 2, true };
            yield return new object[] { Vector3.UnitX, -Vector3.UnitX, false };
            yield return new object[] { Vector3.UnitX, Vector3.UnitY, false };
            yield return new object[] { Vector3.UnitX, Vector3.UnitX * 0.1f, true };
            yield return new object[] { Vector3.UnitY, Vector3.Zero, false };
            yield return new object[] { new Vector3(1, 1, 1), new Vector3(3, 3, 3), true };
        }

    }
}
