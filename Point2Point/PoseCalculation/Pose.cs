using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Point2Point.PoseCalculation
{
    public class Pose : IPose
    {
        public Vector3 Location { get; set; }
        public Vector3 Rotation { get; set; }

        public Pose(Vector3 location, Vector3 rotation)
        {
            Location = location;
            Rotation = rotation;
        }

        public Pose(Vector3 location)
        {
            Location = location;
            Rotation = Vector3.Zero;
        }

        public Pose(IPose poseToCopy)
        {
            if (poseToCopy != null)
            {
                Location = poseToCopy.Location;
                Rotation = poseToCopy.Rotation;
            }
        }

        public override bool Equals(object obj) => obj is Pose pose && Location.Equals(pose.Location) && Rotation.Equals(pose.Rotation);
        public override int GetHashCode() => HashCode.Combine(Location, Rotation);
    }
}
