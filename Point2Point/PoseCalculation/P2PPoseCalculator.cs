using Point2Point.ClearanceHandling;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Point2Point.PoseCalculation
{
    public class P2PPoseCalculator : IPoseCalculator
    {
        private readonly IClearanceHandlingMotionProfile _motionProfile;

        public TimeSpan Created { get; }
        public List<IP2PEdge> Edges { get; }
        public List<IP2PDirectedEdge> DirectedEdges { get; }

        public bool CalculatesLocation => true;
        public bool CalculatesRotation => true;
        public double ClearedLength => _motionProfile?.ClearedLength ?? 0;
        private TimeSpan _timeSpan; 
        
        

        public P2PPoseCalculator(TimeSpan created, IClearanceHandlingMotionProfile motionProfile, IEnumerable<IP2PDirectedEdge> directedEdges)
        {
            Created = created;
            _motionProfile = motionProfile;
            DirectedEdges = directedEdges.ToList();
            Edges = directedEdges.Select(e => e.Edge).ToList();
            _timeSpan = new TimeSpan(0);
        }

        public P2PPoseCalculator(TimeSpan created, IClearanceHandlingMotionProfile motionProfile, params IP2PDirectedEdge[] edges)
            : this(created, motionProfile, edges.ToList())
        {
        }

        public IPose GetPose(TimeSpan time, out double acceleration, out double velocity, out double distance, out IP2PNode currentNode, out int currentEdgeIndex)
        {
            var t = (time - Created).TotalSeconds;
            _motionProfile.GetStatus(t, out acceleration, out velocity, out distance, out currentNode, out currentEdgeIndex);
            var pose = DirectedEdges.GetCurrentPose((float)distance);

            return pose;
        }
        
        public IPose GetPoseDeltaT(TimeSpan deltaT, out double acceleration, out double velocity, out double distance, out IP2PNode currentNode, out int currentEdgeIndex)
        {
            _timeSpan += deltaT; 
            _motionProfile.GetStatus(_timeSpan.TotalSeconds, out acceleration, out velocity, out distance, out currentNode, out currentEdgeIndex);
            var pose = DirectedEdges.GetCurrentPose((float)distance);

            return pose;
        }

        public IPose GetPose(TimeSpan time)
            => GetPose(time, out _, out _, out _, out _, out _);
    }
}
