using System;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectCapsuleBox : DetectFunctor
    {
        private readonly Random random = new Random();

        public CollDetectCapsuleBox() : base((int) PrimitiveType.Capsule, (int) PrimitiveType.Box)
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


            var oldCapsule = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Capsule;
            var newCapsule = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Capsule;
            var oldSeg = new Segment(oldCapsule.Position, oldCapsule.Length * oldCapsule.Orientation.Backward);
            var newSeg = new Segment(newCapsule.Position, newCapsule.Length * newCapsule.Orientation.Backward);

            var radius = oldCapsule.Radius;

            var oldBox = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Box;
            var newBox = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Box;

            var oldDistSq = Distance.SegmentBoxDistanceSq(out var oldSegT, out var oldBoxT0, out var oldBoxT1, out var oldBoxT2, oldSeg, oldBox);
            var newDistSq = Distance.SegmentBoxDistanceSq(out var newSegT, out var newBoxT0, out var newBoxT1, out var newBoxT2, newSeg, newBox);

            if (MathHelper.Min(oldDistSq, newDistSq) < (radius + collTolerance) * (radius + collTolerance))
            {
                var segPos = oldSeg.GetPoint(oldSegT);
                var boxPos = oldBox.GetCentre() + oldBoxT0 * oldBox.Orientation.Right + oldBoxT1 * oldBox.Orientation.Up + oldBoxT2 * oldBox.Orientation.Backward;

                var dist = (float) System.Math.Sqrt(oldDistSq);
                var depth = radius - dist;

                Vector3 dir;

                if (dist > JiggleMath.Epsilon)
                {
                    dir = segPos - boxPos;
                    JiggleMath.NormalizeSafe(ref dir);
                }
                else if ((segPos - oldBox.GetCentre()).LengthSquared() > JiggleMath.Epsilon)
                {
                    dir = segPos - oldBox.GetCentre();
                    JiggleMath.NormalizeSafe(ref dir);
                }
                else
                {
                    dir = Vector3.Transform(Vector3.Backward, Matrix.CreateFromAxisAngle(Vector3.Up, MathHelper.ToRadians(random.Next(360))));
                }

                unsafe
                {
                    var collInfo = new SmallCollPointInfo(boxPos - body0Pos, boxPos - body1Pos, depth);

                    collisionFunctor.CollisionNotify(ref info, ref dir, &collInfo, 1);
                }
            }
        }
    }
}