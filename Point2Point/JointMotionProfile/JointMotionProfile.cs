using System.Collections.Generic;
using System.Linq;

namespace Point2Point.JointMotionProfile
{
    public class JointMotionProfile
    {
        public List<MotionProfileSegment> Segments { get; } = new List<MotionProfileSegment>();

        public void Append(MotionProfileSegment segment, params MotionProfileSegment[] segments)
            => Append(new List<MotionProfileSegment>() { segment }.Concat(segments));

        public void Append(IEnumerable<MotionProfileSegment> segments)
        {
            foreach (var segment in segments)
            {
                segment.Start = Segments.LastOrDefault()?.End ?? 0;
                Segments.Add(segment);
            }

            // UpdateProfile();
        }

        public IEnumerable<MotionProfileSegment> GetEffectiveSegments()
        {
            var minVelocityPoints = Segments
                .SelectMany(s => new[] { new SegmentPoint(s.Start, s.MaximumVelocity), new SegmentPoint(s.End, s.MaximumVelocity) })
                .OrderBy(sp => sp.Distance)
                .GroupBy(sp => sp.Distance)
                .Select(group => new SegmentPoint(group.Key, group.Min(p => p.Velocity)))
                .ToList();

            var effectiveSegments = new List<MotionProfileSegment>();
            for (var i = 0; i < minVelocityPoints.Count - 1; i++)
            {
                var from = minVelocityPoints[i];
                var to = minVelocityPoints[i + 1];
                effectiveSegments.Add(new MotionProfileSegment(from.Distance, to.Distance - from.Distance, from.Velocity));
            }

            return effectiveSegments;
        }

        private void UpdateProfile()
        {
            var effectiveSegments = GetEffectiveSegments();
        }

        private struct SegmentPoint
        {
            public double Distance { get; set; }
            public double Velocity { get; set; }

            public SegmentPoint(double distance, double velocity)
            {
                Distance = distance;
                Velocity = velocity;
            }
        }
    }
}
