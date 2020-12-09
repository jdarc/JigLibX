using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using Microsoft.Xna.Framework;
using Plane = JigLibX.Geometry.Primitives.Plane;

namespace JigLibX.Collision.Detection
{
    public class CollDetectSpherePlane : DetectFunctor
    {
        public CollDetectSpherePlane() : base((int) PrimitiveType.Sphere, (int) PrimitiveType.Plane)
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


            var oldSphere = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Sphere;
            var newSphere = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Sphere;

            var oldPlane = info.Skin1.GetPrimitiveOldWorld(info.IndexPrim1) as Plane;
            var newPlane = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as Plane;

            var newPlaneInvTransform = newPlane.InverseTransformMatrix;
            var oldPlaneInvTransform = oldPlane.InverseTransformMatrix;

            var oldSpherePos = Vector3.Transform(oldSphere.Position, oldPlaneInvTransform);
            var newSpherePos = Vector3.Transform(newSphere.Position, newPlaneInvTransform);


            var oldDist = Distance.PointPlaneDistance(oldSpherePos, oldPlane);
            var newDist = Distance.PointPlaneDistance(newSpherePos, newPlane);

            if (System.Math.Min(newDist, oldDist) > collTolerance + newSphere.Radius) return;


            var oldDepth = oldSphere.Radius - oldDist;


            var worldPos = oldSphere.Position - oldSphere.Radius * oldPlane.Normal;

            unsafe
            {
                var collInfo = new SmallCollPointInfo(worldPos - body0Pos, worldPos - body1Pos, oldDepth);
                collisionFunctor.CollisionNotify(ref info, ref oldPlane.normal, &collInfo, 1);
            }
        }
    }
}