using System.Collections.Generic;
using JigLibX.Geometry;
using JigLibX.Geometry.Primitives;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Collision.Detection
{
    public class CollDetectBoxStaticMesh : DetectFunctor
    {
        public CollDetectBoxStaticMesh() : base((int) PrimitiveType.Box, (int) PrimitiveType.TriangleMesh)
        {
        }

        private static void CollDetectOverlap(ref CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var mesh = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as TriangleMesh;

            var oldBox = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Box;
            var newBox = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Box;

            CollDetectBoxStaticMeshOverlap(oldBox, newBox, mesh, ref info, collTolerance, collisionFunctor);
        }

        private static void CollDetectSweep(ref CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var mesh = info.Skin1.GetPrimitiveNewWorld(info.IndexPrim1) as TriangleMesh;

            var oldBox = info.Skin0.GetPrimitiveOldWorld(info.IndexPrim0) as Box;
            var newBox = info.Skin0.GetPrimitiveNewWorld(info.IndexPrim0) as Box;

            oldBox.GetCentre(out var oldCentre);
            newBox.GetCentre(out var newCentre);

            Vector3.Subtract(ref newCentre, ref oldCentre, out var delta);

            var boxMinLen = 0.5f * System.Math.Min(newBox.SideLengths.X, System.Math.Min(newBox.SideLengths.Y, newBox.SideLengths.Z));
            var nPositions = 1 + (int) (delta.Length() / boxMinLen);

            if (nPositions > 50)
            {
                System.Diagnostics.Debug.WriteLine("Warning - clamping max positions in swept box test");
                nPositions = 50;
            }

            if (nPositions == 1)
            {
                CollDetectBoxStaticMeshOverlap(oldBox, newBox, mesh, ref info, collTolerance, collisionFunctor);
            }
            else
            {
                var bb = BoundingBoxHelper.InitialBox;
                BoundingBoxHelper.AddBox(oldBox, ref bb);
                BoundingBoxHelper.AddBox(newBox, ref bb);
                unsafe
                {
                    var potentialTriangles = stackalloc int[MaxLocalStackTris];
                    {
                        var numTriangles = mesh.GetTrianglesIntersectingtAABox(potentialTriangles, MaxLocalStackTris, ref bb);
                        if (numTriangles <= 0) return;
                        for (var i = 0; i <= nPositions; ++i)
                        {
                            var frac = (float) i / nPositions;
                            Vector3.Multiply(ref delta, frac, out var centre);
                            Vector3.Add(ref centre, ref oldCentre, out centre);

                            var orient = Matrix.Add(Matrix.Multiply(oldBox.Orientation, 1.0f - frac), Matrix.Multiply(newBox.Orientation, frac));
                            var box = new Box(centre - 0.5f * Vector3.TransformNormal(newBox.SideLengths, orient), orient, newBox.SideLengths);
                            
                            CollDetectBoxStaticMeshOverlap(oldBox, box, mesh, ref info, collTolerance, collisionFunctor);
                        }
                    }
                }
            }
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
                CollDetectSweep(ref info, collTolerance, collisionFunctor);
            else
                CollDetectOverlap(ref info, collTolerance, collisionFunctor);
        }

        private static bool AddPoint(List<Vector3> pts, ref Vector3 pt, float combinationDistanceSq)
        {
            for (var i = pts.Count; i-- != 0;)
                if (Distance.PointPointDistanceSq(pts[i], pt) < combinationDistanceSq)
                {
                    pts[i] = 0.5f * (pts[i] + pt);
                    return false;
                }

            pts.Add(pt);
            return true;
        }

        private static int GetBoxTriangleIntersectionPoints(List<Vector3> pts, Box box, Triangle triangle, float combinationDistance)
        {
            box.GetEdges(out var edges);
            box.GetCornerPoints(out var boxPts);

            float tS;


            var point = new Vector3();

            int iEdge;
            for (iEdge = 0; iEdge < 12; ++iEdge)
            {
                var edge = edges[iEdge];
                var seg = new Segment(boxPts[(int) edge.Ind0], boxPts[(int) edge.Ind1] - boxPts[(int) edge.Ind0]);
                if (Intersection.SegmentTriangleIntersection(out tS, out var tv1, out var tv2, seg, triangle))
                {
                    seg.GetPoint(ref point, tS);
                    AddPoint(pts, ref point, combinationDistance * combinationDistance);
                }
            }
            
            for (iEdge = 0; iEdge < 3; ++iEdge)
            {
                var pt0 = triangle.GetPoint(iEdge);
                var pt1 = triangle.GetPoint((iEdge + 1) % 3);

                Vector3.Subtract(ref pt1, ref pt0, out var difference1);
                Vector3.Subtract(ref pt0, ref pt1, out var difference2);

                var s1 = new Segment(ref pt0, ref difference1);
                var s2 = new Segment(ref pt1, ref difference2);

                if (box.SegmentIntersect(out tS, out var pos, out var n, s1))
                    AddPoint(pts, ref pos, combinationDistance * combinationDistance);
                if (box.SegmentIntersect(out tS, out pos, out n, s2))
                    AddPoint(pts, ref pos, combinationDistance * combinationDistance);
            }

            return pts.Count;
        }

        private static bool Disjoint(out float d, ref Vector3 axis, Box box, ref Triangle triangle, float collTolerance)
        {
            box.GetSpan(out var minBox, out var maxBox, ref axis);
            triangle.GetSpan(out var minTri, out var maxTri, ref axis);

            if (minBox > maxTri + collTolerance + JiggleMath.Epsilon || minTri > maxBox + collTolerance + JiggleMath.Epsilon)
            {
                d = 0.0f;
                return true;
            }

            if (maxBox > maxTri && minTri > minBox)
            {
                d = System.Math.Min(maxBox - minTri, maxTri - minBox);
            }
            else if (maxTri > maxBox && minBox > minTri)
            {
                d = System.Math.Min(maxTri - minBox, maxBox - minTri);
            }
            else
            {
                d = maxBox < maxTri ? maxBox : maxTri;
                d -= minBox > minTri ? minBox : minTri;
            }

            return false;
        }

        private static bool DoOverlapBoxTriangleTest(Box oldBox, Box newBox, ref IndexedTriangle triangle, TriangleMesh mesh, ref CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var dirs0 = newBox.Orientation;

            mesh.GetVertex(triangle.GetVertexIndex(0), out var triVec0);
            mesh.GetVertex(triangle.GetVertexIndex(1), out var triVec1);
            mesh.GetVertex(triangle.GetVertexIndex(2), out var triVec2);


            var transformMatrix = mesh.TransformMatrix;
            Vector3.Transform(ref triVec0, ref transformMatrix, out triVec0);
            Vector3.Transform(ref triVec1, ref transformMatrix, out triVec1);
            Vector3.Transform(ref triVec2, ref transformMatrix, out triVec2);

            var tri = new Triangle(ref triVec0, ref triVec1, ref triVec2);


            tri.GetPoint(0, out var pt0);
            tri.GetPoint(1, out var pt1);

            Vector3.Subtract(ref pt1, ref pt0, out var triEdge0);

            tri.GetPoint(2, out var pt2);

            Vector3.Subtract(ref pt2, ref pt1, out var triEdge1);

            Vector3.Subtract(ref pt0, ref pt2, out var triEdge2);

            triEdge0.Normalize();
            triEdge1.Normalize();
            triEdge2.Normalize();
            
            var triNormal = triangle.Plane.Normal;
            var right = dirs0.Right;
            var up = dirs0.Up;
            var backward = dirs0.Backward;

            if (Disjoint(out var testDepth, ref triNormal, newBox, ref tri, collTolerance)) return false;

            var depth = testDepth;
            var N = triNormal;

            if (Disjoint(out testDepth, ref right, newBox, ref tri, collTolerance)) return false;

            if (testDepth < depth)
            {
                depth = testDepth;
                N = right;
            }

            if (Disjoint(out testDepth, ref up, newBox, ref tri, collTolerance)) return false;

            if (testDepth < depth)
            {
                depth = testDepth;
                N = up;
            }

            if (Disjoint(out testDepth, ref backward, newBox, ref tri, collTolerance)) return false;

            if (testDepth < depth)
            {
                depth = testDepth;
                N = backward;
            }

            Vector3.Cross(ref right, ref triEdge0, out var axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }

            Vector3.Cross(ref right, ref triEdge1, out axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }

            Vector3.Cross(ref right, ref triEdge2, out axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }

            Vector3.Cross(ref up, ref triEdge0, out axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }

            Vector3.Cross(ref up, ref triEdge1, out axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }

            Vector3.Cross(ref up, ref triEdge2, out axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }

            Vector3.Cross(ref backward, ref triEdge0, out axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }

            Vector3.Cross(ref backward, ref triEdge1, out axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }

            Vector3.Cross(ref backward, ref triEdge2, out axis);
            if (Disjoint(out testDepth, ref axis, newBox, ref tri, collTolerance)) return false;

            testDepth *= 1.0f / (float) System.Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
            if (testDepth < depth)
            {
                depth = testDepth;
                N = axis;
            }
            
            var D = newBox.GetCentre() - tri.Centre;
            N.Normalize();
            int i;

            if (Vector3.Dot(D, N) < 0.0f) N *= -1;

            var boxOldPos = info.Skin0.Owner?.OldPosition ?? Vector3.Zero;
            var boxNewPos = info.Skin0.Owner?.Position ?? Vector3.Zero;
            var meshPos = info.Skin1.Owner?.OldPosition ?? Vector3.Zero;

            var pts = new List<Vector3>();


            const float combinationDist = 0.05f;
            GetBoxTriangleIntersectionPoints(pts, newBox, tri, depth + combinationDist);


            Vector3.Subtract(ref boxNewPos, ref boxOldPos, out var delta);

            Vector3.Dot(ref delta, ref N, out var oldDepth);
            oldDepth += depth;

            unsafe
            {
                var numPts = pts.Count;
                var collPts = stackalloc SmallCollPointInfo[MaxLocalStackScpi];
                {
                    if (numPts > 0)
                    {
                        if (numPts >= MaxLocalStackScpi) numPts = MaxLocalStackScpi - 1;


                        for (i = 0; i < numPts; ++i)
                        {
                            collPts[i].R0.X = pts[i].X - boxNewPos.X;
                            collPts[i].R0.Y = pts[i].Y - boxNewPos.Y;
                            collPts[i].R0.Z = pts[i].Z - boxNewPos.Z;

                            collPts[i].R1.X = pts[i].X - meshPos.X;
                            collPts[i].R1.Y = pts[i].Y - meshPos.Y;
                            collPts[i].R1.Z = pts[i].Z - meshPos.Z;

                            collPts[i].InitialPenetration = oldDepth;
                        }

                        collisionFunctor.CollisionNotify(ref info, ref N, collPts, numPts);
                        return true;
                    }
                    return false;
                }
            }
        }

        private static bool CollDetectBoxStaticMeshOverlap(Box oldBox, Box newBox, TriangleMesh mesh, ref CollDetectInfo info, float collTolerance, CollisionFunctor collisionFunctor)
        {
            var boxRadius = newBox.GetBoundingRadiusAroundCentre();

            newBox.GetCentre(out var boxCentre);

            var invTransformMatrix = mesh.InverseTransformMatrix;
            Vector3.Transform(ref boxCentre, ref invTransformMatrix, out boxCentre);

            var bb = BoundingBoxHelper.InitialBox;
            BoundingBoxHelper.AddBox(newBox, ref bb);

            unsafe
            {
                var collision = false;
                var potentialTriangles = stackalloc int[MaxLocalStackTris];
                {

                    var numTriangles = mesh.GetTrianglesIntersectingtAABox(potentialTriangles, MaxLocalStackTris, ref bb);

                    for (var iTriangle = 0; iTriangle < numTriangles; ++iTriangle)
                    {
                        var meshTriangle = mesh.GetTriangle(potentialTriangles[iTriangle]);


                        var dist = meshTriangle.Plane.DotCoordinate(boxCentre);


                        if (dist > boxRadius || dist < -boxRadius) continue;

                        if (DoOverlapBoxTriangleTest(oldBox, newBox, ref meshTriangle, mesh, ref info, collTolerance, collisionFunctor)) collision = true;
                    }
                }
                return collision;
            }
        }
    }
}