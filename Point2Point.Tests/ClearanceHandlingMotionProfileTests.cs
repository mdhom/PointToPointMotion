using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Point2Point.Mathematics.ExtendedP2P;
using Point2Point;
using Point2Point.ClearanceHandling;
using Point2Point.Tests.Model;

namespace Point2Point.Tests
{
    [TestClass]
    public class ClearanceHandlingMotionProfileTests
    {
        private Map _map;
        private List<DirectedEdge> _edges;
        private MotionParameter _parameter;
        private readonly double _vMax = 800;

        [TestInitialize]
        public void Initialize()
        {
            const int numNodes = 8;
            _map = new Map();
            for (int i = 0; i < numNodes; i++)
            {
                _map.AddNodeAt(i * 1000, 0, 0);
                if (i > 0)
                {
                    _map.Connect(i, i + 1);
                }
            }

            _edges = new List<DirectedEdge>();
            for (int i = 1; i < numNodes; i++)
            {
                _edges.Add(new DirectedEdge(_map.GetEdge(i)));
            }

            _parameter = new MotionParameter(2000, -2000, 500, -500);
        }

        [TestMethod]
        public void ProfileWithClearanceGrantingsIsConsistent()
        {
            const double delta = 0.01;
            const double timeStep = 0.1;
            var profile = new ClearanceHandlingMotionProfile(TimeSpan.Zero, _edges, _parameter, _vMax, null);
            var sb = new StringBuilder();

            var aLC = 0.0;
            var vLC = 0.0;
            var sLC = 0.0;
            var maxDeltaA = _parameter.PositiveJerk * timeStep * 1.1; // max allowed delta a and 10% tolerance
            var maxDeltaV = _parameter.MaximumAcceleration * timeStep * 1.1; // max allowed delta v and 10% tolerance
            var maxDeltaS = _vMax * timeStep * 1.1; // max allowed delta v and 10% tolerance
            for (double t = 0.0; t < 2000; t += timeStep)
            {
                profile.GetStatus(t, out var a, out var v, out var s, out _, out var currentEdgeIndex);
                sb.AppendLine($"{t};{a};{v};{s};{profile.ClearedLength};{currentEdgeIndex * 1000}");

                Assert.IsTrue(Math.Abs(a - aLC) < maxDeltaA);
                Assert.IsTrue(Math.Abs(v - vLC) < maxDeltaV);
                Assert.IsTrue(Math.Abs(s - sLC) < maxDeltaS);

                aLC = a;
                vLC = v;
                sLC = s;

                var ts = TimeSpan.FromSeconds(t);

                if (profile.ClearedLength < 1000 && t > 1)
                {
                    // grant after some time and never granted before
                    Assert.AreEqual(0.0, profile.ClearedLength, delta);
                    profile.ClearanceGranted(1, ts);
                    Assert.AreEqual(1000.0, profile.ClearedLength, delta);
                }
                else if (profile.ClearedLength < 2000 && t > 5)
                {
                    // grant after beeing in stillstand again but granted before
                    Assert.AreEqual(1000.0, profile.ClearedLength, delta);
                    Assert.IsTrue(profile.ClearanceGranted(2, ts));
                    Assert.IsFalse(profile.ClearanceGranted(2, ts)); // clearing twice must not cause error!
                    Assert.AreEqual(2000.0, profile.ClearedLength, delta);
                }
                else if (profile.ClearedLength < 3000 && t > 7.5)
                {
                    // grant during brake phase
                    Assert.AreEqual(2000.0, profile.ClearedLength, delta);
                    profile.ClearanceGranted(3, ts);
                    Assert.AreEqual(3000.0, profile.ClearedLength, delta);
                }
                else if (profile.ClearedLength < 4000 && t > 8.4)
                {
                    // grant during acceleration phase
                    Assert.AreEqual(3000.0, profile.ClearedLength, delta);
                    profile.ClearanceGranted(4, ts);
                    Assert.AreEqual(4000.0, profile.ClearedLength, delta);
                }
                else if (profile.ClearedLength < 5000 && t > 9.7)
                {
                    // grant during constant phase
                    Assert.AreEqual(4000.0, profile.ClearedLength, delta);
                    profile.ClearanceGranted(5, ts);
                    Assert.AreEqual(5000.0, profile.ClearedLength, delta);
                }
                else if (profile.ClearedLength < 6000 && t > 11.6)
                {
                    // grant multiple edges at once
                    Assert.AreEqual(5000.0, profile.ClearedLength, delta);
                    profile.ClearanceGranted(7, ts);
                    Assert.AreEqual(7000.0, profile.ClearedLength, delta);
                }
            }

            profile.GetStatus(20.0, out var aOverstepped, out var vOverstepped, out var sOverstepped, out _, out var currentEdgeIndexOverstepped);

            // StringBuilder output can be visualized in excel sheet
            var output = sb.ToString();
        }

    }
}
