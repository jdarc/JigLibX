using JigLibX.Geometry.Primitives;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectSphereHeightmap : DetectFunctor
    {
        public CollDetectSphereHeightmap() : base((int) PrimitiveType.Sphere, (int) PrimitiveType.Heightmap)
        {
        }

        public override void CollDetect(CollDetectInfo infoOrig, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var info = infoOrig;
            if (info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0).Type == Type1)
            {
                var skinSwap = info.Skin0;
                info.Skin0 = info.Skin1;
                info.Skin1 = skinSwap;
                var primSwap = info.IndexPrim0;
                info.IndexPrim0 = info.IndexPrim1;
                info.IndexPrim1 = primSwap;
            }

            var body0Pos = info.Skin0.Owner?.OldPosition ?? Vector3.Zero;
            var body1Pos = info.Skin1.Owner?.OldPosition ?? Vector3.Zero;


            var oldSphere = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Sphere;
            var newSphere = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Sphere;

            var oldHeightmap = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Heightmap;
            var newHeightmap = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Heightmap;

            newHeightmap.GetHeightAndNormal(out var newDist, out var normal, newSphere.Position);
            if (newDist < collTolerance + newSphere.Radius)
            {
                var oldDist = oldHeightmap.GetHeight(oldSphere.Position);
                var depth = oldSphere.Radius - oldDist;


                var oldPt = oldSphere.Position - oldSphere.Radius * normal;
                unsafe
                {
                    var ptInfo = new SmallCollPointInfo(oldPt - body0Pos, oldPt - body1Pos, depth);

                    collisionFunctor.CollisionNotify(ref info, ref normal, &ptInfo, 1);
                }
            }
        }
    }
}