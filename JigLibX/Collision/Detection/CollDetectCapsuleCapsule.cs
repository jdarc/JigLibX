using System;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectCapsuleCapsule : DetectFunctor
    {
        private Random random = new Random();

        public CollDetectCapsuleCapsule() : base((int) PrimitiveType.Capsule, (int) PrimitiveType.Capsule)
        {
        }

        public override void CollDetect(CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var body0Pos = info.Skin0.Owner?.OldPosition ?? Vector3.Zero;
            var body1Pos = info.Skin1.Owner?.OldPosition ?? Vector3.Zero;


            var oldCapsule0 = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Capsule;
            var newCapsule0 = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Capsule;
            var oldSeg0 = new Segment(oldCapsule0.Position, oldCapsule0.Length * oldCapsule0.Orientation.Backward);
            var newSeg0 = new Segment(newCapsule0.Position, newCapsule0.Length * newCapsule0.Orientation.Backward);

            var oldCapsule1 = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Capsule;
            var newCapsule1 = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Capsule;
            var oldSeg1 = new Segment(oldCapsule1.Position, oldCapsule1.Length * oldCapsule1.Orientation.Backward);
            var newSeg1 = new Segment(newCapsule1.Position, newCapsule1.Length * newCapsule1.Orientation.Backward);

            var radSum = newCapsule0.Radius + newCapsule1.Radius;

            var oldDistSq = Distance.SegmentSegmentDistanceSq(out var oldt0, out var oldt1, oldSeg0, oldSeg1);
            var newDistSq = Distance.SegmentSegmentDistanceSq(out var newt0, out var newt1, newSeg0, newSeg1);

            if (System.Math.Min(oldDistSq, newDistSq) < (radSum + collTolerance) * (radSum + collTolerance))
            {
                var pos0 = oldSeg0.GetPoint(oldt0);
                var pos1 = oldSeg1.GetPoint(oldt1);

                var delta = pos0 - pos1;

                var dist = (float) System.Math.Sqrt(oldDistSq);
                var depth = radSum - dist;

                if (dist > JiggleMath.Epsilon)
                    delta /= dist;
                else

                    delta = Vector3.TransformNormal(Vector3.Backward, Matrix.CreateFromAxisAngle(Vector3.Up, MathHelper.ToRadians(random.Next(360))));

                var worldPos = pos1 + (oldCapsule1.Radius - 0.5f * depth) * delta;

                unsafe
                {
                    var collInfo = new SmallCollPointInfo(worldPos - body0Pos, worldPos - body1Pos, depth);
                    collisionFunctor.CollisionNotify(ref info, ref delta, &collInfo, 1);
                }
            }
        }
    }
}