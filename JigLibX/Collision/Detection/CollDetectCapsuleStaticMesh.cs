using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectCapsuleStaticMesh : DetectFunctor
    {
        public CollDetectCapsuleStaticMesh() : base((int) PrimitiveType.Capsule, (int) PrimitiveType.TriangleMesh)
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

            if (info.Skin0.CollisionSystem != null && info.Skin0.CollisionSystem.UseSweepTests)
                CollDetectSweep(info, collTolerance, collisionFunctor);
            else
                CollDetectOverlap(info, collTolerance, collisionFunctor);
        }

        private void CollDetectCapsuleStaticMeshOverlap(Capsule oldCapsule, Capsule newCapsule, TriangleMesh mesh, CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var body0Pos = info.Skin0.Owner?.OldPosition ?? Vector3.Zero;
            var body1Pos = info.Skin1.Owner?.OldPosition ?? Vector3.Zero;

            var capsuleTolR = collTolerance + newCapsule.Radius;
            var capsuleTolR2 = capsuleTolR * capsuleTolR;

            var collNormal = Vector3.Zero;

            var bb = BoundingBoxHelper.InitialBox;
            BoundingBoxHelper.AddCapsule(newCapsule, ref bb);

            unsafe
            {
                var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
                var potentialTriangles = stackalloc int[MaxLocalStackTris];
                {
                    {
                        var numCollPts = 0;

                        var numTriangles = mesh.GetTrianglesIntersectingtAABox(potentialTriangles, MaxLocalStackTris, ref bb);

                        var capsuleStart = newCapsule.Position;
                        var capsuleEnd = newCapsule.GetEnd();
                        var meshInvTransform = mesh.InverseTransformMatrix;

                        var meshSpaceCapsuleStart = Vector3.Transform(capsuleStart, meshInvTransform);
                        var meshSpaceCapsuleEnd = Vector3.Transform(capsuleEnd, meshInvTransform);

                        for (var iTriangle = 0; iTriangle < numTriangles; ++iTriangle)
                        {
                            var meshTriangle = mesh.GetTriangle(potentialTriangles[iTriangle]);


                            var distToStart = meshTriangle.Plane.DotCoordinate(meshSpaceCapsuleStart);
                            var distToEnd = meshTriangle.Plane.DotCoordinate(meshSpaceCapsuleEnd);


                            if (distToStart > capsuleTolR && distToEnd > capsuleTolR || distToStart < -capsuleTolR && distToEnd < -capsuleTolR) continue;


                            meshTriangle.GetVertexIndices(out var i0, out var i1, out var i2);

                            mesh.GetVertex(i0, out var triVec0);
                            mesh.GetVertex(i1, out var triVec1);
                            mesh.GetVertex(i2, out var triVec2);


                            var transformMatrix = mesh.TransformMatrix;
                            Vector3.Transform(ref triVec0, ref transformMatrix, out triVec0);
                            Vector3.Transform(ref triVec1, ref transformMatrix, out triVec1);
                            Vector3.Transform(ref triVec2, ref transformMatrix, out triVec2);
                            var triangle = new Triangle(ref triVec0, ref triVec1, ref triVec2);

                            var seg = new Segment(capsuleStart, capsuleEnd - capsuleStart);

                            var d2 = Distance.SegmentTriangleDistanceSq(out var tS, out var tT0, out var tT1, seg, triangle);

                            if (d2 < capsuleTolR2)
                            {
                                var oldCapsuleStart = oldCapsule.Position;
                                var oldCapsuleEnd = oldCapsule.GetEnd();
                                var oldSeg = new Segment(oldCapsuleStart, oldCapsuleEnd - oldCapsuleStart);
                                d2 = Distance.SegmentTriangleDistanceSq(out tS, out tT0, out tT1, oldSeg, triangle);

                                var dist = (float) System.Math.Sqrt(d2);
                                var depth = oldCapsule.Radius - dist;
                                var pt = triangle.GetPoint(tT0, tT1);
                                var collisionN = d2 > JiggleMath.Epsilon ? JiggleMath.NormalizeSafe(oldSeg.GetPoint(tS) - pt) : meshTriangle.Plane.Normal;
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
            var oldCapsule = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Capsule;
            var newCapsule = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Capsule;


            var mesh = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as TriangleMesh;

            CollDetectCapsuleStaticMeshOverlap(oldCapsule, newCapsule, mesh, info, collTolerance, collisionFunctor);
        }

        private void CollDetectCapsulseStaticMeshSweep(Capsule oldCapsule, Capsule newCapsule, TriangleMesh mesh, CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var delta = newCapsule.Position - oldCapsule.Position;
            if (delta.LengthSquared() < 0.25f * newCapsule.Radius * newCapsule.Radius)
            {
                CollDetectCapsuleStaticMeshOverlap(oldCapsule, newCapsule, mesh, info, collTolerance, collisionFunctor);
            }
            else
            {
                var capsuleLen = oldCapsule.Length;
                var capsuleRadius = oldCapsule.Radius;

                var nSpheres = 2 + (int) (capsuleLen / (2.0f * oldCapsule.Radius));
                for (var iSphere = 0; iSphere < nSpheres; ++iSphere)
                {
                    var offset = iSphere * capsuleLen / (nSpheres - 1.0f);
                    var oldSphere = new BoundingSphere(oldCapsule.Position + oldCapsule.Orientation.Backward * offset, capsuleRadius);
                    var newSphere = new BoundingSphere(newCapsule.Position + newCapsule.Orientation.Backward * offset, capsuleRadius);
                    CollDetectSphereStaticMesh.CollDetectSphereStaticMeshSweep(oldSphere, newSphere, mesh, info, collTolerance, collisionFunctor);
                }
            }
        }

        private void CollDetectSweep(CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var oldCapsule = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Capsule;
            var newCapsule = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Capsule;


            var mesh = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as TriangleMesh;

            CollDetectCapsulseStaticMeshSweep(oldCapsule, newCapsule, mesh, info, collTolerance, collisionFunctor);
        }
    }
}