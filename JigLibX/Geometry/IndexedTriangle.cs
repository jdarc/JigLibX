using System.Collections.Generic;
using Microsoft.Xna.Framework;
using System.Diagnostics;

namespace JigLibX.Geometry
{
    public struct IndexedTriangle
    {
        internal int counter;

        private int vertexIndices0;

        private int vertexIndices1;
        private int vertexIndices2;
        private int convexFlags;

        public IndexedTriangle(int i0, int i1, int i2, List<Vector3> vertexArray)
        {
            counter = 0;
            vertexIndices0 = i0;
            vertexIndices1 = i1;
            vertexIndices2 = i2;

            convexFlags = unchecked((ushort) ~0);
            Plane = new Plane(vertexArray[i0], vertexArray[i1], vertexArray[i2]);
        }

        public IndexedTriangle(int i0, int i1, int i2, Vector3[] vertexArray)
        {
            counter = 0;

            vertexIndices0 = i0;
            vertexIndices1 = i1;
            vertexIndices2 = i2;

            convexFlags = unchecked((ushort) ~0);
            Plane = new Plane(vertexArray[i0], vertexArray[i1], vertexArray[i2]);
        }

        public void SetVertexIndices(int i0, int i1, int i2, List<Vector3> vertexArray)
        {
            vertexIndices0 = i0;
            vertexIndices1 = i1;
            vertexIndices2 = i2;

            Plane = new Plane(vertexArray[i0], vertexArray[i1], vertexArray[i2]);
        }

        public void SetVertexIndices(int i0, int i1, int i2, Vector3[] vertexArray)
        {
            vertexIndices0 = i0;
            vertexIndices1 = i1;
            vertexIndices2 = i2;

            Plane = new Plane(vertexArray[i0], vertexArray[i1], vertexArray[i2]);
        }

        public void GetVertexIndices(out int i0, out int i1, out int i2)
        {
            i0 = vertexIndices0;
            i1 = vertexIndices1;
            i2 = vertexIndices2;
        }

        public int GetVertexIndex(int iCorner)
        {
            switch (iCorner)
            {
                case 0:
                    return vertexIndices0;
                case 1:
                    return vertexIndices1;
                case 2:
                    return vertexIndices2;
                default:
                    Debug.Assert(false);
                    return vertexIndices0;
            }
        }

        public Plane Plane { get; private set; }

        public bool IsEdgeConvex(int iEdge)
        {
            return 0 != (convexFlags & (1 << iEdge));
        }

        public void SetEdgeConvex(int iEdge, bool convex)
        {
            if (convex)
                convexFlags |= (ushort) (1 << iEdge);
            else
                convexFlags &= (ushort) ~(1 << iEdge);
        }

        public bool IsPointConvex(int iPoint)
        {
            return 0 != (convexFlags & (1 << (iPoint + 3)));
        }

        public void SetPointConvex(int iPoint, bool convex)
        {
            if (convex)
                convexFlags |= (ushort) (1 << (iPoint + 3));
            else
                convexFlags &= (ushort) ~(1 << (iPoint + 3));
        }
    }
}