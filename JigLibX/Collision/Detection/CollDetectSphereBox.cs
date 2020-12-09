using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectSphereBox : DetectFunctor
    {
        public CollDetectSphereBox() : base((int) PrimitiveType.Sphere, (int) PrimitiveType.Box)
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

            var oldBox = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Box;
            var newBox = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Box;

            var oldDist = oldBox.GetDistanceToPoint(out var oldBoxPoint, oldSphere.Position);
            var newDist = newBox.GetDistanceToPoint(out var newBoxPoint, newSphere.Position);


            var oldDepth = oldSphere.Radius - oldDist;
            var newDepth = newSphere.Radius - newDist;

            if (System.Math.Max(oldDepth, newDepth) > -collTolerance)
            {
                Vector3 dir;
                if (oldDist < -JiggleMath.Epsilon)
                {
                    dir = oldBoxPoint - oldSphere.Position - oldBoxPoint;
                    JiggleMath.NormalizeSafe(ref dir);
                }
                else if (oldDist > JiggleMath.Epsilon)
                {
                    dir = oldSphere.Position - oldBoxPoint;
                    JiggleMath.NormalizeSafe(ref dir);
                }
                else
                {
                    dir = oldSphere.Position - oldBox.GetCentre();
                    JiggleMath.NormalizeSafe(ref dir);
                }

                unsafe
                {
                    var collInfo = new SmallCollPointInfo(oldBoxPoint - body0Pos, oldBoxPoint - body1Pos, oldDepth);


                    collisionFunctor.CollisionNotify(ref info, ref dir, &collInfo, 1);
                }
            }
        }
    }
}