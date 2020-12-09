using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using Microsoft.Xna.Framework;
using Plane = JigLibX.Geometry.Primitives.Plane;

namespace JigLibX.Collision.Detection
{
    public class CollDetectCapsulePlane : DetectFunctor
    {
        public CollDetectCapsulePlane() : base((int) PrimitiveType.Capsule, (int) PrimitiveType.Plane)
        {
        }

        public override void CollDetect(CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
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


            var oldCapsule = (Capsule) info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0);
            var newCapsule = (Capsule) info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0);

            var oldPlane = (Plane) info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1);
            var newPlane = (Plane) info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1);

            var newPlaneInvTransform = newPlane.InverseTransformMatrix;
            var oldPlaneInvTransform = oldPlane.InverseTransformMatrix;

            unsafe
            {
                var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
                {
                    var numCollPts = 0;


                    {
                        var oldCapsuleStartPos = Vector3.Transform(oldCapsule.Position, oldPlaneInvTransform);
                        var newCapsuleStartPos = Vector3.Transform(newCapsule.Position, newPlaneInvTransform);

                        var oldDist = Distance.PointPlaneDistance(oldCapsuleStartPos, oldPlane);
                        var newDist = Distance.PointPlaneDistance(newCapsuleStartPos, newPlane);

                        if (MathHelper.Min(newDist, oldDist) < collTolerance + newCapsule.Radius)
                        {
                            var oldDepth = oldCapsule.Radius - oldDist;

                            var worldPos = oldCapsule.Position - oldCapsule.Radius * oldPlane.Normal;


                            collPts[numCollPts].R0 = worldPos - body0Pos;
                            collPts[numCollPts].R1 = worldPos - body1Pos;
                            collPts[numCollPts++].InitialPenetration = oldDepth;
                        }
                    }


                    {
                        var oldCapsuleEndPos = Vector3.Transform(oldCapsule.GetEnd(), oldPlaneInvTransform);
                        var newCapsuleEndPos = Vector3.Transform(newCapsule.GetEnd(), newPlaneInvTransform);
                        var oldDist = Distance.PointPlaneDistance(oldCapsuleEndPos, oldPlane);
                        var newDist = Distance.PointPlaneDistance(newCapsuleEndPos, newPlane);

                        if (System.Math.Min(newDist, oldDist) < collTolerance + newCapsule.Radius)
                        {
                            var oldDepth = oldCapsule.Radius - oldDist;

                            var worldPos = oldCapsule.GetEnd() - oldCapsule.Radius * oldPlane.Normal;


                            collPts[numCollPts].R0 = worldPos - body0Pos;
                            collPts[numCollPts].R1 = worldPos - body1Pos;
                            collPts[numCollPts++].InitialPenetration = oldDepth;
                        }

                        if (numCollPts > 0)
                            collisionFunctor.CollisionNotify(ref info, ref oldPlane.normal, collPts, numCollPts);
                    }
                }
            }
        }
    }
}