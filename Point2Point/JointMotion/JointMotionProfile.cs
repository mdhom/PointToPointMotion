using System.Collections.Generic;
using System.Linq;

namespace Point2Point.JointMotion
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
                segment.Start = Segments.Any() ? Segments.Max(s => s.End) : 0;
                Segments.Add(segment);
            }

            // UpdateProfile();
        }

        public void Insert(MotionProfileSegment segment)
        {
            Segments.Add(segment);
        }

        public List<MotionProfileSegment> GetEffectiveSegments()
        {
            var distanceValues = Segments
                .SelectMany(s => new[] { s.Start, s.End })
                .Distinct()
                .OrderBy(d => d)
                .ToList();

            var minPoints = new List<SegmentPoint>();
            foreach (var distance in distanceValues)
            {
                var containingSegments = Segments.Where(s => s.Contains(distance));
                if (containingSegments.Any())
                {
                    var minAllowedVelocity = containingSegments.Min(s => s.MaximumVelocity);
                    minPoints.Add(new SegmentPoint(distance, minAllowedVelocity));
                }
            }

            var effectiveSegments = new List<MotionProfileSegment>();
            for (var i = 0; i < minPoints.Count; i++)
            {
                var from = minPoints[i];
                var endIndex = minPoints.FindIndex(i + 1, sp => sp.Velocity != from.Velocity);
                if (endIndex == -1)
                {
                    // segment with other velocity further found
                    effectiveSegments.Add(new MotionProfileSegment(from.Distance, Segments.Max(s => s.End) - from.Distance, from.Velocity));
                    return effectiveSegments;
                }
                else
                {
                    var to = minPoints[endIndex];
                    effectiveSegments.Add(new MotionProfileSegment(from.Distance, to.Distance - from.Distance, from.Velocity));
                }

                i = endIndex - 1;
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
