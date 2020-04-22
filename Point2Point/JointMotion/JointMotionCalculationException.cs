using System;
using System.Runtime.Serialization;

namespace Point2Point.JointMotion
{
    [Serializable]
    public class JointMotionCalculationException : Exception
    {
        public JointMotionCalculationException()
        {
        }

        public JointMotionCalculationException(string message)
            : base(message)
        {
        }

        public JointMotionCalculationException(string message, Exception innerException)
            : base(message, innerException)
        {
        }

        protected JointMotionCalculationException(SerializationInfo info, StreamingContext context)
            : base(info, context)
        {
        }
    }
}
