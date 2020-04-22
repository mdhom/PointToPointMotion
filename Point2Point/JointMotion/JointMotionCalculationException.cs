using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.Text;
using System.Threading.Tasks;

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
