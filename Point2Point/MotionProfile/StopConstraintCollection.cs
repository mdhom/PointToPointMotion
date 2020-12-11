using System.Collections.Generic;
using System.Linq;
using MoreLinq;

namespace Point2Point.JointMotion
{
    public class StopConstraintCollection : List<StopConstraint>
    {
        #region Constructors

        public StopConstraintCollection()
        {
        }

        public StopConstraintCollection(IEnumerable<StopConstraint> constraints)
            : base(constraints)
        {
        }

        public StopConstraintCollection(StopConstraint constraint, params StopConstraint[] constraints)
            : base(new List<StopConstraint>() {constraint}.Concat(constraints))
        {
        }

        #endregion
    }
}

