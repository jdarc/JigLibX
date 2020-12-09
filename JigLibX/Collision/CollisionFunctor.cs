using Microsoft.Xna.Framework;

namespace JigLibX.Collision
{
    public abstract class CollisionFunctor
    {
        public abstract unsafe void CollisionNotify(ref CollDetectInfo collDetectInfo, ref Vector3 dirToBody0, SmallCollPointInfo* pointInfos, int numCollPts);
    }
}