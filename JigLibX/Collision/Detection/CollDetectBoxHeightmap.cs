using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectBoxHeightmap : DetectFunctor
    {
        public CollDetectBoxHeightmap() : base((int) PrimitiveType.Box, (int) PrimitiveType.Heightmap)
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


            var oldBox = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Box;
            var newBox = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Box;

            var oldHeightmap = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Heightmap;
            var newHeightmap = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Heightmap;

            oldBox.GetCornerPoints(out var oldPts);
            newBox.GetCornerPoints(out var newPts);

            unsafe
            {
                var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
                {
                    var numCollPts = 0;

                    var collNormal = Vector3.Zero;

                    for (var i = 0; i < 8; ++i)
                    {
                        var newPt = newPts[i];
                        newHeightmap.GetHeightAndNormal(out var newDist, out var normal, newPt);

                        if (newDist < collTolerance)
                        {
                            var oldPt = oldPts[i];
                            var oldDist = oldHeightmap.GetHeight(oldPt);

                            Vector3.Subtract(ref oldPt, ref body0Pos, out var pt0);
                            Vector3.Subtract(ref oldPt, ref body1Pos, out var pt1);
                            if (numCollPts < MaxLocalStackScpi)
                            {
                                collPts[numCollPts].R0 = pt0;
                                collPts[numCollPts].R1 = pt1;
                                collPts[numCollPts++].InitialPenetration = -oldDist;
                            }

                            Vector3.Add(ref collNormal, ref normal, out collNormal);
                        }
                    }

                    if (numCollPts > 0)
                    {
                        JiggleMath.NormalizeSafe(ref collNormal);
                        collisionFunctor.CollisionNotify(ref info, ref collNormal, collPts, numCollPts);
                    }
                }
            }
        }
    }
}