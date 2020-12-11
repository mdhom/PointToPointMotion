using Microsoft.VisualStudio.TestTools.UnitTesting;
using Point2Point.JointMotion;

namespace Point2Point.Tests
{
    [TestClass]
    public class ConstraintsCollectionTests
    {
        private const double _delta = 0.00001;

        [TestMethod]
        public void GetEffectiveSegmentsWorks()
        {
            // Arrange
            var constraints = new VelocityConstraintsCollection(
                new VelocityConstraint(0,    1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(500,  1000, 200));

            // Act
            var effectiveSegments = constraints.GetEffectiveConstraints();

            // Assert
            Assert.AreEqual(3, effectiveSegments.Count);
            Assert.AreEqual(0,    effectiveSegments[0].Start, _delta);
            Assert.AreEqual(500,  effectiveSegments[0].Length, _delta);
            Assert.AreEqual(500,  effectiveSegments[0].MaximumVelocity, _delta);

            Assert.AreEqual(500,  effectiveSegments[1].Start, _delta);
            Assert.AreEqual(1000, effectiveSegments[1].Length, _delta);
            Assert.AreEqual(200,  effectiveSegments[1].MaximumVelocity, _delta);

            Assert.AreEqual(1500, effectiveSegments[2].Start, _delta);
            Assert.AreEqual(500,  effectiveSegments[2].Length, _delta);
            Assert.AreEqual(400,  effectiveSegments[2].MaximumVelocity, _delta);
        }

        [TestMethod]
        public void GapsBetweenSegmentsAreFilledUp()
        {
            // Arrange
            var constraints = new VelocityConstraintsCollection(
                new VelocityConstraint(0, 1000, 500),
                new VelocityConstraint(1000, 1000, 400),
                new VelocityConstraint(2500, 1000, 200)); // start 500mm after end of last segment (2000)

            // Act
            var effectiveSegments = constraints.GetEffectiveConstraints();

            // Assert
            Assert.AreEqual(3, effectiveSegments.Count);

            Assert.AreEqual(0,      effectiveSegments[0].Start, _delta);
            Assert.AreEqual(1000,   effectiveSegments[0].Length, _delta);
            Assert.AreEqual(500,    effectiveSegments[0].MaximumVelocity, _delta);

            Assert.AreEqual(1000,   effectiveSegments[1].Start, _delta);
            Assert.AreEqual(1500,   effectiveSegments[1].Length, _delta); // length is filled up! original length of second segment was 1000
            Assert.AreEqual(400,    effectiveSegments[1].MaximumVelocity, _delta);

            Assert.AreEqual(2500,   effectiveSegments[2].Start, _delta);
            Assert.AreEqual(1000,   effectiveSegments[2].Length, _delta);
            Assert.AreEqual(200,    effectiveSegments[2].MaximumVelocity, _delta);
        }
    }
}
