using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using Microsoft.Xna.Framework;
using JPlane = JigLibX.Geometry.Primitives.Plane;

namespace JigLibX.Collision.Detection
{
    public class CollDetectBoxPlane : DetectFunctor
    {
        private readonly Vector3[] _oldTransPts = new Vector3[8];

        public CollDetectBoxPlane() : base((int) PrimitiveType.Box, (int) PrimitiveType.Plane)
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

            var oldBox = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Box;
            var newBox = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Box;

            var oldPlane = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as JPlane;
            var newPlane = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as JPlane;

            var newPlaneInvTransform = newPlane.InverseTransformMatrix;
            var newBoxCen = Vector3.Transform(newBox.GetCentre(), newPlaneInvTransform);


            var centreDist = Distance.PointPlaneDistance(newBoxCen, newPlane);
            if (centreDist > collTolerance + newBox.GetBoundingRadiusAroundCentre()) return;

            var oldPlaneInvTransform = oldPlane.InverseTransformMatrix;

            newBox.GetCornerPoints(out var newPts);
            oldBox.GetCornerPoints(out var oldPts);

            unsafe
            {
                var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
                {
                    var numCollPts = 0;

                    for (var i = 0; i < 8; ++i)
                    {
                        Vector3.Transform(ref oldPts[i], ref oldPlaneInvTransform, out _oldTransPts[i]);
                        Vector3.Transform(ref newPts[i], ref newPlaneInvTransform, out newPts[i]);

                        var oldDepth = -Distance.PointPlaneDistance(ref _oldTransPts[i], oldPlane);
                        var newDepth = -Distance.PointPlaneDistance(ref newPts[i], newPlane);

                        if (MathHelper.Max(oldDepth, newDepth) > -collTolerance)
                            if (numCollPts < MaxLocalStackScpi)
                            {
                                collPts[numCollPts].R0 = oldPts[i] - body0Pos;
                                collPts[numCollPts].R1 = oldPts[i] - body1Pos;
                                collPts[numCollPts++].InitialPenetration = oldDepth;
                            }
                    }

                    if (numCollPts > 0)
                        collisionFunctor.CollisionNotify(ref info, ref oldPlane.normal, collPts, numCollPts);
                }
            }
        }
    }
}