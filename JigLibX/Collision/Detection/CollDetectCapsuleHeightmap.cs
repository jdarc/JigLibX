using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectCapsuleHeightmap : DetectFunctor
    {
        public CollDetectCapsuleHeightmap() : base((int) PrimitiveType.Capsule, (int) PrimitiveType.Heightmap)
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

            var oldHeightmap = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Heightmap;
            var newHeightmap = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Heightmap;

            unsafe
            {
#if USE_STACKALLOC
                var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
#else
                var collPtArray = SCPIStackAlloc();
                fixed (SmallCollPointInfo* collPts = collPtArray)
#endif
                {
                    var numCollPts = 0;
                    var averageNormal = Vector3.Zero;


                    {
                        oldHeightmap.GetHeightAndNormal(out var oldDist, out var normal, oldCapsule.Position);
                        newHeightmap.GetHeightAndNormal(out var newDist, out normal, newCapsule.Position);

                        if (MathHelper.Min(newDist, oldDist) < collTolerance + newCapsule.Radius)
                        {
                            var oldDepth = oldCapsule.Radius - oldDist;

                            var worldPos = oldCapsule.Position - oldCapsule.Radius * normal;
                            if (numCollPts < MaxLocalStackScpi)
                            {
                                collPts[numCollPts].R0 = worldPos - body0Pos;
                                collPts[numCollPts].R1 = worldPos - body1Pos;
                                collPts[numCollPts++].InitialPenetration = oldDepth;
                            }

                            averageNormal += normal;
                        }
                    }

                    {
                        var oldEnd = oldCapsule.GetEnd();
                        var newEnd = newCapsule.GetEnd();
                        oldHeightmap.GetHeightAndNormal(out var oldDist, out var normal, oldEnd);
                        newHeightmap.GetHeightAndNormal(out var newDist, out normal, newEnd);
                        if (MathHelper.Min(newDist, oldDist) < collTolerance + newCapsule.Radius)
                        {
                            var oldDepth = oldCapsule.Radius - oldDist;

                            var worldPos = oldEnd - oldCapsule.Radius * normal;
                            if (numCollPts < MaxLocalStackScpi)
                            {
                                collPts[numCollPts].R0 = worldPos - body0Pos;
                                collPts[numCollPts].R1 = worldPos - body1Pos;
                                collPts[numCollPts++].InitialPenetration = oldDepth;
                            }

                            averageNormal += normal;
                        }
                    }

                    if (numCollPts > 0)
                    {
                        JiggleMath.NormalizeSafe(ref averageNormal);
                        collisionFunctor.CollisionNotify(ref info, ref averageNormal, collPts, numCollPts);
                    }
                }
#if !USE_STACKALLOC
                FreeStackAlloc(collPtArray);
#endif
            }
        }
    }
}