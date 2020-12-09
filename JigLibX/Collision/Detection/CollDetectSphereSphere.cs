using System;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectSphereSphere : DetectFunctor
    {
        private Random random = new Random();

        public CollDetectSphereSphere() : base((int) PrimitiveType.Sphere, (int) PrimitiveType.Sphere)
        {
        }

        public override void CollDetect(CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var body0Pos = info.Skin0.Owner?.OldPosition ?? Vector3.Zero;
            var body1Pos = info.Skin1.Owner?.OldPosition ?? Vector3.Zero;


            var oldSphere0 = (Sphere) info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0);
            var newSphere0 = (Sphere) info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0);
            var oldSphere1 = (Sphere) info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1);
            var newSphere1 = (Sphere) info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1);

            var oldDelta = oldSphere0.Position - oldSphere1.Position;
            var newDelta = newSphere0.Position - oldSphere1.Position;

            var oldDistSq = oldDelta.LengthSquared();
            var newDistSq = newDelta.LengthSquared();

            var radSum = newSphere0.Radius + newSphere1.Radius;

            if (System.Math.Min(oldDistSq, newDistSq) < (radSum + collTolerance) * (radSum + collTolerance))
            {
                var oldDist = (float) System.Math.Sqrt(oldDistSq);
                var depth = radSum - oldDist;

                if (oldDist > JiggleMath.Epsilon)
                    oldDelta /= oldDist;
                else

                    oldDelta = Vector3.TransformNormal(Vector3.Backward, Matrix.CreateFromAxisAngle(Vector3.Up, MathHelper.ToRadians(random.Next(360))));

                var worldPos = oldSphere1.Position + (oldSphere1.Radius - 0.5f * depth) * oldDelta;

                unsafe
                {
                    var collInfo = new SmallCollPointInfo(worldPos - body0Pos, worldPos - body1Pos, depth);

                    collisionFunctor.CollisionNotify(ref info, ref oldDelta, &collInfo, 1);
                }
            }
        }
    }
}