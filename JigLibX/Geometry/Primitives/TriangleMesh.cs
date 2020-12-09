using System.Collections.Generic;
using JigLibX.Collision.Detection;
using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Geometry.Primitives
{
    public class TriangleMesh : Primitive
    {
        private int maxTrianglesPerCell;
        private float minCellSize;

        public TriangleMesh() : base((int) PrimitiveType.TriangleMesh)
        {
        }

        public void CreateMesh(List<Vector3> vertices, List<TriangleVertexIndices> triangleVertexIndices, int maxTrianglesPerCell, float minCellSize)
        {
            var numVertices = vertices.Count;

            Octree.Clear(true);
            Octree.AddTriangles(vertices, triangleVertexIndices);
            Octree.BuildOctree(maxTrianglesPerCell, minCellSize);

            this.maxTrianglesPerCell = maxTrianglesPerCell;
            this.minCellSize = minCellSize;
        }

        public override void GetBoundingBox(out AABox box)
        {
            box = Octree.BoundingBox.Clone() as AABox;
            box.Transform = Transform;
        }

        private Matrix transformMatrix;
        private Matrix invTransform;

        public override Transform Transform
        {
            get => base.Transform;
            set
            {
                base.Transform = value;
                transformMatrix = transform.Orientation;
                transformMatrix.Translation = transform.Position;
                invTransform = Matrix.Invert(transformMatrix);
            }
        }

        public override Matrix TransformMatrix => transformMatrix;

        public override Matrix InverseTransformMatrix => invTransform;

        public Octree Octree { get; private set; } = new Octree();

        public int GetNumTriangles()
        {
            return Octree.NumTriangles;
        }

        public IndexedTriangle GetTriangle(int iTriangle)
        {
            return Octree.GetTriangle(iTriangle);
        }

        public Vector3 GetVertex(int iVertex)
        {
            return Octree.GetVertex(iVertex);
        }

        public void GetVertex(int iVertex, out Vector3 result)
        {
            result = Octree.GetVertex(iVertex);
        }

        public unsafe int GetTrianglesIntersectingtAABox(int* triangles, int maxTriangles, ref BoundingBox bb)
        {
            var rotBB = bb;

            var bbCorner = new Vector3();

            for (var a = 0; a < 2; a++)
            for (var b = 0; b < 2; b++)
            for (var c = 0; c < 2; c++)
            {
                bbCorner.X = a == 0 ? bb.Min.X : bb.Max.X;
                bbCorner.Y = b == 0 ? bb.Min.Y : bb.Max.Y;
                bbCorner.Z = c == 0 ? bb.Min.Z : bb.Max.Z;
                var bbCornerT = Vector3.Transform(bbCorner, invTransform);

                BoundingBoxHelper.AddPoint(ref bbCornerT, ref rotBB);
            }

            return Octree.GetTrianglesIntersectingtAABox(triangles, maxTriangles, ref rotBB);
        }

        public override Primitive Clone() => new TriangleMesh {Octree = Octree, Transform = Transform};

        public override bool SegmentIntersect(out float frac, out Vector3 pos, out Vector3 normal, Segment seg)
        {
            seg.Origin = Vector3.Transform(seg.Origin, invTransform);
            seg.Delta = Vector3.TransformNormal(seg.Delta, invTransform);


            var segBox = BoundingBoxHelper.InitialBox;
            BoundingBoxHelper.AddSegment(seg, ref segBox);

            unsafe
            {
                var potentialTriangles = stackalloc int[DetectFunctor.MaxLocalStackTris];
                {
                    var numTriangles = GetTrianglesIntersectingtAABox(potentialTriangles, DetectFunctor.MaxLocalStackTris, ref segBox);

                    pos = Vector3.Zero;
                    normal = Vector3.Zero;

                    var bestFrac = float.MaxValue;
                    for (var iTriangle = 0; iTriangle < numTriangles; ++iTriangle)
                    {
                        var meshTriangle = GetTriangle(potentialTriangles[iTriangle]);
                        var tri = new Triangle(GetVertex(meshTriangle.GetVertexIndex(0)), GetVertex(meshTriangle.GetVertexIndex(1)), GetVertex(meshTriangle.GetVertexIndex(2)));

                        if (Intersection.SegmentTriangleIntersection(out var thisFrac, out var tv1, out var tv2, seg, tri))
                            if (thisFrac < bestFrac)
                            {
                                bestFrac = thisFrac;

                                pos = Vector3.Transform(seg.GetPoint(thisFrac), transformMatrix);
                                normal = Vector3.TransformNormal(meshTriangle.Plane.Normal, transformMatrix);
                            }
                    }

                    frac = bestFrac;
                    if (bestFrac < float.MaxValue)
                    {
                        // DetectFunctor.FreeStackAlloc(potTriArray);
                        return true;
                    }
                    else
                    {
                        // DetectFunctor.FreeStackAlloc(potTriArray);
                        return false;
                    }
                }
            }
        }

        public override float GetVolume() => 0.0f;

        public override float GetSurfaceArea() => 0.0f;

        public override void GetMassProperties(PrimitiveProperties primitiveProperties, out float mass, out Vector3 centerOfMass, out Matrix inertiaTensor)
        {
            mass = 0.0f;
            centerOfMass = Vector3.Zero;
            inertiaTensor = Matrix.Identity;
        }
    }
}