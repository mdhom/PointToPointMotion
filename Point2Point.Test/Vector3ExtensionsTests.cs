using Xunit;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using System.Collections;

namespace Point2Point.Tests
{
    
    public class Vector3ExtensionsTests
    {
        [Theory]
        [ClassData(typeof(TestDataAngleBetween))]
        public void AngleBetweenWorks(Vector3 v0, Vector3 v1, double expectedAngle)
        {
            var angle = v0.AngleBetween(v1);

            Assert.Equal(expectedAngle, angle, 3);
        }

        private class TestDataAngleBetween : IEnumerable<object[]>
        {

            private readonly List<object[]> _data = new List<object[]>
            {
                 new object[] { Vector3.UnitX, Vector3.UnitY, 90.0 },
                 new object[] { Vector3.UnitX, Vector3.UnitZ, 90.0 },
                 new object[] { Vector3.UnitX, Vector3.UnitX, 0.0 },
                new object[] { Vector3.UnitX, new Vector3(1, 1, 0), 45.0 },
            };

            public IEnumerator<object[]> GetEnumerator() => _data.GetEnumerator();

            IEnumerator IEnumerable.GetEnumerator() => _data.GetEnumerator();
        }

        [Fact]
        public void AbsWorks()
        {
            // Arrange
            var v = new Vector3(-1, -2, -3);

            // Act
            var absV = v.Abs();

            // Assert
            Assert.Equal(1f, absV.X);
            Assert.Equal(2f, absV.Y);
            Assert.Equal(3f, absV.Z);
        }

        [Theory]
        [ClassData(typeof(TestDataSameDirection))]
        public void HasSameDirectionWorks(Vector3 vector1, Vector3 vector2, bool expectedResult)
        {
            var result = vector1.HasSameDirection(vector2);
            Assert.Equal(expectedResult, result);
        }


        private class TestDataSameDirection : IEnumerable<object[]>
        {

            private readonly List<object[]> _data = new List<object[]>
            {
                new object[] { Vector3.UnitX, Vector3.UnitX * 2, true },
                new object[] { Vector3.UnitX, -Vector3.UnitX, false },
                new object[] { Vector3.UnitX, Vector3.UnitY, false },
                new object[] { Vector3.UnitX, Vector3.UnitX* 0.1f, true },
                new object[] { Vector3.UnitY, Vector3.Zero, false },
                new object[] { new Vector3(1, 1, 1), new Vector3(3, 3, 3), true },
            };

            public IEnumerator<object[]> GetEnumerator() => _data.GetEnumerator();

            IEnumerator IEnumerable.GetEnumerator() => _data.GetEnumerator();
        }
    }
}
