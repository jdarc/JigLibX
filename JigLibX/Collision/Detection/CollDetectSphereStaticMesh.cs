using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectSphereStaticMesh : DetectFunctor
    {
        public CollDetectSphereStaticMesh() : base((int) PrimitiveType.Sphere, (int) PrimitiveType.TriangleMesh)
        {
        }

        public static void CollDetectSphereStaticMeshOverlap(BoundingSphere oldSphere, BoundingSphere newSphere, TriangleMesh mesh, CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var body0Pos = info.Skin0.Owner?.OldPosition ?? Vector3.Zero;
            var body1Pos = info.Skin1.Owner?.OldPosition ?? Vector3.Zero;

            var sphereTolR = collTolerance + newSphere.Radius;
            var sphereTolR2 = sphereTolR * sphereTolR;

            unsafe
            {
                var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
                var potentialTriangles = stackalloc int[MaxLocalStackTris];
                {
                    {
                        var numCollPts = 0;

                        var collNormal = Vector3.Zero;

                        var bb = BoundingBoxHelper.InitialBox;
                        BoundingBoxHelper.AddSphere(newSphere, ref bb);
                        var numTriangles = mesh.GetTrianglesIntersectingtAABox(potentialTriangles, MaxLocalStackTris, ref bb);


                        var newSphereCen = Vector3.Transform(newSphere.Center, mesh.InverseTransformMatrix);
                        var oldSphereCen = Vector3.Transform(oldSphere.Center, mesh.InverseTransformMatrix);

                        for (var iTriangle = 0; iTriangle < numTriangles; ++iTriangle)
                        {
                            var meshTriangle = mesh.GetTriangle(potentialTriangles[iTriangle]);
                            var distToCentre = meshTriangle.Plane.DotCoordinate(newSphereCen);


                            if (distToCentre < -sphereTolR || distToCentre > sphereTolR) continue;

                            meshTriangle.GetVertexIndices(out var i0, out var i1, out var i2);

                            var triangle = new Triangle(mesh.GetVertex(i0), mesh.GetVertex(i1), mesh.GetVertex(i2));

                            var newD2 = Distance.PointTriangleDistanceSq(out var s, out var t, newSphereCen, triangle);

                            if (newD2 < sphereTolR2)
                            {
                                var oldD2 = Distance.PointTriangleDistanceSq(out s, out t, oldSphereCen, triangle);
                                var dist = (float) System.Math.Sqrt(oldD2);
                                var depth = oldSphere.Radius - dist;

                                var triPointSTNorm = oldSphereCen - triangle.GetPoint(s, t);
                                JiggleMath.NormalizeSafe(ref triPointSTNorm);

                                var collisionN = dist > float.Epsilon ? triPointSTNorm : triangle.Normal;


                                var pt = oldSphere.Center - oldSphere.Radius * collisionN;

                                if (numCollPts < MaxLocalStackScpi)
                                {
                                    collPts[numCollPts].R0 = pt - body0Pos;
                                    collPts[numCollPts].R1 = pt - body1Pos;
                                    collPts[numCollPts++].InitialPenetration = depth;
                                }

                                collNormal += collisionN;
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

        private void CollDetectOverlap(CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var oldSphere = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Sphere;
            var newSphere = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Sphere;
            var oldBSphere = new BoundingSphere(oldSphere.Position, oldSphere.Radius);
            var newBSphere = new BoundingSphere(newSphere.Position, newSphere.Radius);


            var mesh = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as TriangleMesh;

            CollDetectSphereStaticMeshOverlap(oldBSphere, newBSphere, mesh, info, collTolerance, collisionFunctor);
        }

        internal static void CollDetectSphereStaticMeshSweep(BoundingSphere oldSphere, BoundingSphere newSphere, TriangleMesh mesh, CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var delta = newSphere.Center - oldSphere.Center;
            if (delta.LengthSquared() < 0.25f * newSphere.Radius * newSphere.Radius)
            {
                CollDetectSphereStaticMeshOverlap(oldSphere, newSphere, mesh, info, collTolerance, collisionFunctor);
            }
            else
            {
                var body0Pos = info.Skin0.Owner?.OldPosition ?? Vector3.Zero;
                var body1Pos = info.Skin1.Owner?.OldPosition ?? Vector3.Zero;

                var sphereTolR = collTolerance + oldSphere.Radius;
                var sphereToR2 = sphereTolR * sphereTolR;

                var collNormal = Vector3.Zero;

                var bb = BoundingBoxHelper.InitialBox;
                BoundingBoxHelper.AddSphere(oldSphere, ref bb);
                BoundingBoxHelper.AddSphere(newSphere, ref bb);


                var newSphereCen = Vector3.Transform(newSphere.Center, mesh.InverseTransformMatrix);
                var oldSphereCen = Vector3.Transform(oldSphere.Center, mesh.InverseTransformMatrix);

                unsafe
                {
                    var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
                    var potentialTriangles = stackalloc int[MaxLocalStackTris];
                    {
                        {
                            var numCollPts = 0;

                            var numTriangles = mesh.GetTrianglesIntersectingtAABox(potentialTriangles, MaxLocalStackTris, ref bb);

                            for (var iTriangle = 0; iTriangle < numTriangles; ++iTriangle)
                            {
                                var meshTriangle = mesh.GetTriangle(potentialTriangles[iTriangle]);
                                var distToCentreOld = meshTriangle.Plane.DotCoordinate(oldSphereCen);
                                if (distToCentreOld <= 0.0f) continue;


                                var distToCentreNew = meshTriangle.Plane.DotCoordinate(newSphereCen);
                                if (distToCentreNew > sphereTolR) continue;

                                meshTriangle.GetVertexIndices(out var i0, out var i1, out var i2);

                                var triangle = new Triangle(mesh.GetVertex(i0), mesh.GetVertex(i1), mesh.GetVertex(i2));


                                var d2 = Distance.PointTriangleDistanceSq(out var s, out var t, oldSphereCen, triangle);

                                if (d2 < sphereToR2)
                                {
                                    var dist = (float) System.Math.Sqrt(d2);
                                    var depth = oldSphere.Radius - dist;
                                    var triangleN = triangle.Normal;
                                    var normSafe = oldSphereCen - triangle.GetPoint(s, t);

                                    JiggleMath.NormalizeSafe(ref normSafe);

                                    var collisionN = dist > float.Epsilon ? normSafe : triangleN;

                                    var pt = oldSphere.Center - oldSphere.Radius * collisionN;
                                    if (numCollPts < MaxLocalStackScpi)
                                    {
                                        collPts[numCollPts].R0 = pt - body0Pos;
                                        collPts[numCollPts].R1 = pt - body1Pos;
                                        collPts[numCollPts++].InitialPenetration = depth;
                                    }

                                    collNormal += collisionN;
                                }
                                else if (distToCentreNew < distToCentreOld)
                                {
                                    if (Intersection.SweptSphereTriangleIntersection(out var pt, out var N, out var depth, oldSphere, newSphere, triangle, distToCentreOld, distToCentreNew, Intersection.EdgesToTest.EdgeAll, Intersection.CornersToTest.CornerAll))
                                    {
                                        var dist = (float) System.Math.Sqrt(d2);
                                        var depth2 = oldSphere.Radius - dist;
                                        var triangleN = triangle.Normal;
                                        var normSafe = oldSphereCen - triangle.GetPoint(s, t);
                                        JiggleMath.NormalizeSafe(ref normSafe);
                                        var collisionN = dist > JiggleMath.Epsilon ? normSafe : triangleN;

                                        var pt2 = oldSphere.Center - oldSphere.Radius * collisionN;
                                        if (numCollPts < MaxLocalStackScpi)
                                        {
                                            collPts[numCollPts].R0 = pt2 - body0Pos;
                                            collPts[numCollPts].R1 = pt2 - body1Pos;
                                            collPts[numCollPts++].InitialPenetration = depth;
                                        }

                                        collNormal += collisionN;
                                    }
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

        private void CollDetectSweep(CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var oldSphere = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Sphere;
            var newSphere = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Sphere;
            var oldBSphere = new BoundingSphere(oldSphere.Position, oldSphere.Radius);
            var newBSphere = new BoundingSphere(newSphere.Position, newSphere.Radius);


            var mesh = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as TriangleMesh;

            CollDetectSphereStaticMeshSweep(oldBSphere, newBSphere, mesh, info, collTolerance, collisionFunctor);
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

            if (info.Skin0.CollisionSystem != null && info.Skin0.CollisionSystem.UseSweepTests)
                CollDetectSweep(info, collTolerance, collisionFunctor);
            else
                CollDetectOverlap(info, collTolerance, collisionFunctor);
        }
    }
}