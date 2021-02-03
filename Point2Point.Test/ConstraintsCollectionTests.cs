using Point2Point.JointMotion;
using Xunit;

namespace Point2Point.Tests
{
    public class ConstraintsCollectionTests
    {
        private const int _delta = 5;

        [Fact]
        public void GetEffectiveSegmentsWorks()
        {
            // Arrange
            var constraints = new ConstraintsCollection(
                new VelocityConstraint(0,    1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(500,  1000, 200));

            // Act
            var effectiveSegments = constraints.GetEffectiveConstraints();

            // Assert
            Assert.Equal(3, effectiveSegments.Count);
            Assert.Equal(0,    effectiveSegments[0].Start, _delta);
            Assert.Equal(500,  effectiveSegments[0].Length, _delta);
            Assert.Equal(500,  effectiveSegments[0].MaximumVelocity, _delta);

            Assert.Equal(500,  effectiveSegments[1].Start, _delta);
            Assert.Equal(1000, effectiveSegments[1].Length, _delta);
            Assert.Equal(200,  effectiveSegments[1].MaximumVelocity, _delta);

            Assert.Equal(1500, effectiveSegments[2].Start, _delta);
            Assert.Equal(500,  effectiveSegments[2].Length, _delta);
            Assert.Equal(400,  effectiveSegments[2].MaximumVelocity, _delta);
        }

        [Fact]
        public void GapsBetweenSegmentsAreFilledUp()
        {
            // Arrange
            var constraints = new ConstraintsCollection(
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(2500, 1000, 200)); // start 500mm after end of last segment (2000)

            // Act
            var effectiveSegments = constraints.GetEffectiveConstraints();

            // Assert
            Assert.Equal(3, effectiveSegments.Count);

            Assert.Equal(0,      effectiveSegments[0].Start, _delta);
            Assert.Equal(1000,   effectiveSegments[0].Length, _delta);
            Assert.Equal(500,    effectiveSegments[0].MaximumVelocity, _delta);

            Assert.Equal(1000,   effectiveSegments[1].Start, _delta);
            Assert.Equal(1500,   effectiveSegments[1].Length, _delta); // length is filled up! original length of second segment was 1000
            Assert.Equal(400,    effectiveSegments[1].MaximumVelocity, _delta);

            Assert.Equal(2500,   effectiveSegments[2].Start, _delta);
            Assert.Equal(1000,   effectiveSegments[2].Length, _delta);
            Assert.Equal(200,    effectiveSegments[2].MaximumVelocity, _delta);
        }
    }
}
