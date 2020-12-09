using System;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectSphereCapsule : DetectFunctor
    {
        private Random random = new Random();

        public CollDetectSphereCapsule() : base((int) PrimitiveType.Sphere, (int) PrimitiveType.Capsule)
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

            var oldCapsule = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Capsule;
            var newCapsule = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Capsule;

            var oldSeg = new Segment(oldCapsule.Position, oldCapsule.Length * oldCapsule.Orientation.Backward);
            var newSeg = new Segment(oldCapsule.Position, newCapsule.Length * newCapsule.Orientation.Backward);

            var radSum = newCapsule.Radius + newSphere.Radius;

            var oldDistSq = Distance.PointSegmentDistanceSq(out var oldt, oldSphere.Position, oldSeg);
            var newDistSq = Distance.PointSegmentDistanceSq(out var newt, newSphere.Position, newSeg);

            if (MathHelper.Min(oldDistSq, newDistSq) < (radSum + collTolerance) * (radSum + collTolerance))
            {
                var segPos = oldSeg.GetPoint(oldt);
                var delta = oldSphere.Position - segPos;

                var dist = (float) System.Math.Sqrt(oldDistSq);
                var depth = radSum - dist;

                if (dist > JiggleMath.Epsilon)
                    delta /= dist;
                else

                    delta = Vector3.TransformNormal(Vector3.Backward, Matrix.CreateFromAxisAngle(Vector3.Up, MathHelper.ToRadians(random.Next(360))));

                var worldPos = segPos + (oldCapsule.Radius - 0.5f * depth) * delta;
                unsafe
                {
                    var collInfo = new SmallCollPointInfo(worldPos - body0Pos, worldPos - body1Pos, depth);

                    collisionFunctor.CollisionNotify(ref info, ref delta, &collInfo, 1);
                }
            }
        }
    }
}