using System.Collections.Generic;
using JigLibX.Collision;
using Microsoft.Xna.Framework;

namespace JigLibX.Physics
{
    public class BasicCollisionFunctor : CollisionFunctor
    {
        private List<CollisionInfo> colls;

        public BasicCollisionFunctor(List<CollisionInfo> colls)
        {
            this.colls = colls;
        }

        public override unsafe void CollisionNotify(ref CollDetectInfo collDetectInfo, ref Vector3 dirToBody0, SmallCollPointInfo* pointInfos, int numCollPts)
        {
            CollisionInfo info;

            var skin0 = collDetectInfo.Skin0;
            var skin1 = collDetectInfo.Skin1;


            if (skin0?.Owner != null)
            {
                var generateContactPoints = skin0.OnCollisionEvent(skin0, skin1);
                if (skin1 != null) generateContactPoints &= skin1.OnCollisionEvent(skin1, skin0);

                if (generateContactPoints)
                {
                    info = CollisionInfo.GetCollisionInfo(collDetectInfo, dirToBody0, pointInfos, numCollPts);
                    colls.Add(info);
                    skin0.Collisions.Add(info);

                    if (skin1?.Owner != null) skin1.Collisions.Add(info);
                }
            }
            else if (skin1?.Owner != null)
            {
                var generateContactPoints = skin1.OnCollisionEvent(skin1, skin0);
                if (skin0 != null) generateContactPoints &= skin0.OnCollisionEvent(skin0, skin1);

                if (generateContactPoints)
                {
                    info = CollisionInfo.GetCollisionInfo(collDetectInfo, -dirToBody0, pointInfos, numCollPts);
                    colls.Add(info);
                    skin1.Collisions.Add(info);
                    if (skin0?.Owner != null) skin0.Collisions.Add(info);
                }
            }
            else
            {
                System.Diagnostics.Debug.WriteLine("Collision detected with both skin bodies null.");
            }
        }
    }
}