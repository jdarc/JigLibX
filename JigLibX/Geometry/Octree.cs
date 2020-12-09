using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;
using System.Diagnostics;
using JigLibX.Geometry.Primitives;

namespace JigLibX.Geometry
{
    public class Octree
    {
        [Flags]
        internal enum EChild
        {
            XP = 0x1,
            YP = 0x2,
            ZP = 0x4,
            PPP = XP | YP | ZP,
            PPM = XP | YP,
            PMP = XP | ZP,
            PMM = XP,
            MPP = YP | ZP,
            MPM = YP,
            MMP = ZP,
            MMM = 0x0
        }

        private struct Node
        {
            public ushort[] nodeIndices;
            public int[] triIndices;
            public BoundingBox box;
        }

        private class BuildNode
        {
            public int childType;
            public List<int> nodeIndices = new List<int>();
            public List<int> triIndices = new List<int>();
            public BoundingBox box;
        };

        private Vector3[] positions;
        private BoundingBox[] triBoxes;
        private TriangleVertexIndices[] tris;
        private Node[] nodes;
        private BoundingBox rootNodeBox;
        private ushort[] nodeStack;

        public Octree()
        {
        }

        public void Clear(bool NOTUSED)
        {
            positions = null;
            triBoxes = null;
            tris = null;
            nodes = null;
            BoundingBox = null;
        }

        public void AddTriangles(List<Vector3> _positions, List<TriangleVertexIndices> _tris)
        {
            positions = new Vector3[_positions.Count];
            _positions.CopyTo(positions);


            tris = new TriangleVertexIndices[_tris.Count];
            _tris.CopyTo(tris);
        }

        public void BuildOctree(int _maxTrisPerCellNOTUSED, float _minCellSizeNOTUSED)
        {
            triBoxes = new BoundingBox[tris.Length];


            rootNodeBox = new BoundingBox(new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity), new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity));


            for (var i = 0; i < tris.Length; i++)
            {
                triBoxes[i].Min = Vector3.Min(positions[tris[i].I0], Vector3.Min(positions[tris[i].I1], positions[tris[i].I2]));
                triBoxes[i].Max = Vector3.Max(positions[tris[i].I0], Vector3.Max(positions[tris[i].I1], positions[tris[i].I2]));


                rootNodeBox.Min = Vector3.Min(rootNodeBox.Min, triBoxes[i].Min);
                rootNodeBox.Max = Vector3.Max(rootNodeBox.Max, triBoxes[i].Max);
            }

            BoundingBox = new AABox(rootNodeBox.Min, rootNodeBox.Max);

            var buildNodes = new List<BuildNode>();
            buildNodes.Add(new BuildNode());
            buildNodes[0].box = rootNodeBox;

            var children = new BoundingBox[8];
            for (var triNum = 0; triNum < tris.Length; triNum++)
            {
                var nodeIndex = 0;
                var box = rootNodeBox;

                while (box.Contains(triBoxes[triNum]) == ContainmentType.Contains)
                {
                    var childCon = -1;
                    for (var i = 0; i < 8; ++i)
                    {
                        children[i] = CreateAABox(box, (EChild) i);
                        if (children[i].Contains(triBoxes[triNum]) == ContainmentType.Contains)
                        {
                            childCon = i;
                            break;
                        }
                    }


                    if (childCon == -1)
                    {
                        buildNodes[nodeIndex].triIndices.Add(triNum);
                        break;
                    }
                    else
                    {
                        var childIndex = -1;
                        for (var index = 0; index < buildNodes[nodeIndex].nodeIndices.Count; ++index)
                            if (buildNodes[buildNodes[nodeIndex].nodeIndices[index]].childType == childCon)
                            {
                                childIndex = index;
                                break;
                            }

                        if (childIndex == -1)
                        {
                            var parentNode = buildNodes[nodeIndex];
                            var newNode = new BuildNode();
                            newNode.childType = childCon;
                            newNode.box = children[childCon];
                            buildNodes.Add(newNode);

                            nodeIndex = buildNodes.Count - 1;
                            box = children[childCon];
                            parentNode.nodeIndices.Add(nodeIndex);
                        }
                        else
                        {
                            nodeIndex = buildNodes[nodeIndex].nodeIndices[childIndex];
                            box = children[childCon];
                        }
                    }
                }
            }

            Debug.Assert(buildNodes.Count < 0xFFFF);


            nodes = new Node[buildNodes.Count];
            nodeStack = new ushort[buildNodes.Count];
            for (var i = 0; i < nodes.Length; i++)
            {
                nodes[i].nodeIndices = new ushort[buildNodes[i].nodeIndices.Count];
                for (var index = 0; index < nodes[i].nodeIndices.Length; ++index)
                    nodes[i].nodeIndices[index] = (ushort) buildNodes[i].nodeIndices[index];

                nodes[i].triIndices = new int[buildNodes[i].triIndices.Count];
                buildNodes[i].triIndices.CopyTo(nodes[i].triIndices);
                nodes[i].box = buildNodes[i].box;
            }

            buildNodes = null;
        }

        public Octree(List<Vector3> _positions, List<TriangleVertexIndices> _tris)
        {
            AddTriangles(_positions, _tris);
            BuildOctree(16, 1.0f);
        }

        private BoundingBox CreateAABox(BoundingBox aabb, EChild child)
        {
            var dims = 0.5f * (aabb.Max - aabb.Min);
            var offset = new Vector3();

            switch (child)
            {
                case EChild.PPP:
                    offset = new Vector3(1, 1, 1);
                    break;
                case EChild.PPM:
                    offset = new Vector3(1, 1, 0);
                    break;
                case EChild.PMP:
                    offset = new Vector3(1, 0, 1);
                    break;
                case EChild.PMM:
                    offset = new Vector3(1, 0, 0);
                    break;
                case EChild.MPP:
                    offset = new Vector3(0, 1, 1);
                    break;
                case EChild.MPM:
                    offset = new Vector3(0, 1, 0);
                    break;
                case EChild.MMP:
                    offset = new Vector3(0, 0, 1);
                    break;
                case EChild.MMM:
                    offset = new Vector3(0, 0, 0);
                    break;

                default:
                    Debug.WriteLine("Octree.CreateAABox  got impossible child");


                    break;
            }

            var result = new BoundingBox();
            result.Min = aabb.Min + new Vector3(offset.X * dims.X, offset.Y * dims.Y, offset.Z * dims.Z);
            result.Max = result.Min + dims;


            var extra = 0.00001f;

            result.Min -= extra * dims;
            result.Max += extra * dims;

            return result;
        }

        private void GatherTriangles(int _nodeIndex, ref List<int> _tris)
        {
            _tris.AddRange(nodes[_nodeIndex].triIndices);


            var numChildren = nodes[_nodeIndex].nodeIndices.Length;
            for (var i = 0; i < numChildren; ++i)
            {
                int childNodeIndex = nodes[_nodeIndex].nodeIndices[i];
                GatherTriangles(childNodeIndex, ref _tris);
            }
        }

        public unsafe int GetTrianglesIntersectingtAABox(int* triangles, int maxTriangles, ref BoundingBox testBox)
        {
            if (nodes.Length == 0) return 0;
            var curStackIndex = 0;
            var endStackIndex = 1;
            nodeStack[0] = 0;

            var triCount = 0;

            while (curStackIndex < endStackIndex)
            {
                var nodeIndex = nodeStack[curStackIndex];
                curStackIndex++;
                if (nodes[nodeIndex].box.Contains(testBox) != ContainmentType.Disjoint)
                {
                    for (var i = 0; i < nodes[nodeIndex].triIndices.Length; ++i)
                        if (triBoxes[nodes[nodeIndex].triIndices[i]].Contains(testBox) != ContainmentType.Disjoint)
                            if (triCount < maxTriangles)
                                triangles[triCount++] = nodes[nodeIndex].triIndices[i];

                    var numChildren = nodes[nodeIndex].nodeIndices.Length;
                    for (var i = 0; i < numChildren; ++i) nodeStack[endStackIndex++] = nodes[nodeIndex].nodeIndices[i];
                }
            }

            return triCount;
        }

        public AABox BoundingBox { get; private set; }

        public IndexedTriangle GetTriangle(int _index)
        {
            return new IndexedTriangle(tris[_index].I0, tris[_index].I1, tris[_index].I2, positions);
        }

        public Vector3 GetVertex(int iVertex)
        {
            return positions[iVertex];
        }

        public void GetVertex(int iVertex, out Vector3 result)
        {
            result = positions[iVertex];
        }

        public int NumTriangles => tris.Length;
    }
#if OLD_OCTREE
    public class Octree
    {

        #region Octree Cell
        
        
        
        
        
        
        struct Cell
        {
            
            
            
            
            internal enum EChild
            {
                PPP,
                PPM,
                PMP,
                PMM,
                MPP,
                MPM,
                MMP,
                MMM,
                NumChildren
            }

            
            internal int[] mChildCellIndices;

            
            internal List<int> mTriangleIndices;

            
            internal AABox mAABox;

            
            
            
            
            public Cell(AABox aabb)
            {
                mAABox = aabb;
                mTriangleIndices = new List<int>();
                mChildCellIndices = new int[NumChildren];

                Clear();
            }

            
            
            
            public void Clear()
            {
                for (int i = 0; i < NumChildren; i++)
                    mChildCellIndices[i] = -1;

                mTriangleIndices.Clear();

            }

            
            
            
            public bool IsLeaf
            {
                get { return mChildCellIndices[0] == -1; }
            }


        }
        #endregion

        #region private fields

        private const int NumChildren = (int)Cell.EChild.NumChildren;

        
        
        private List<Octree.Cell> cells;

        
        private List<Vector3> vertices;
        
        private List<IndexedTriangle> triangles;

        private AABox boundingBox = new AABox();

        
        
        
        private Stack<int> mCellsToTest;

        
        
        private int testCounter;

        #endregion

        
        
        
        
        public Octree()
        {
            cells = new List<Cell>();
            vertices = new List<Vector3>();
            triangles = new List<IndexedTriangle>();
            mCellsToTest = new Stack<int>();
        }

        
        
        
        
        
        
        public void Clear(bool freeMemory)
        {
            cells.Clear();
            vertices.Clear();
            triangles.Clear();
        }

        public AABox BoundingBox
        {
            get { return this.boundingBox; }
        }

        
        
        
        
        
        
        
        public void AddTriangles(List<Vector3> vertices, List<TriangleVertexIndices> triangleVertexIndices)
        {
            NewOctree test = new NewOctree(vertices, triangleVertexIndices);

            this.vertices.Clear();
            this.triangles.Clear();
            this.cells.Clear();

            int numTriangles = triangleVertexIndices.Count;

            this.vertices = vertices;

            for (int iTriangle = 0; iTriangle < numTriangles; iTriangle++)
            {
                int i0 = triangleVertexIndices[iTriangle].I0;
                int i1 = triangleVertexIndices[iTriangle].I1;
                int i2 = triangleVertexIndices[iTriangle].I2;

                
                
                

                Vector3 dr1 = vertices[i1] - vertices[i0];
                Vector3 dr2 = vertices[i2] - vertices[i0];
                Vector3 N = Vector3.Cross(dr1, dr2);

                float NLen = N.Length();

                
                
                if (NLen > JiggleMath.Epsilon)
                {
                    IndexedTriangle tri = new IndexedTriangle();
                    tri.SetVertexIndices(i0, i1, i2, vertices);

                    triangles.Add(tri);

                    
                }
            }
        }

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        public void BuildOctree(int maxTrianglesPerCell, float minCellSize)
        {
            boundingBox.Clear();

            for (int i = 0; i < vertices.Count; i++)
                boundingBox.AddPoint(vertices[i]);

            
            cells.Clear();

            

            Octree.Cell rootCell = new Octree.Cell(boundingBox);

            cells.Add(rootCell);
            int numTriangles = triangles.Count;

            

            for (int i = 0; i < numTriangles; i++)
                rootCell.mTriangleIndices.Add(i);

            
            
            
            
            Stack<int> cellsToProcess = new Stack<int>();
            cellsToProcess.Push(0);

            
            
            while (cellsToProcess.Count != 0)
            {
                int cellIndex = cellsToProcess.Pop();

                if ((cells[cellIndex].mTriangleIndices.Count <= maxTrianglesPerCell) ||
                     (cells[cellIndex].mAABox.GetRadiusAboutCentre() < minCellSize))
                    continue;

                
                for (int iChild = 0; iChild < NumChildren; iChild++)
                {
                    cells[cellIndex].mChildCellIndices[iChild] = cells.Count;
                    cellsToProcess.Push(cells.Count);

                    Octree.Cell childCell =
 new Octree.Cell(CreateAABox(cells[cellIndex].mAABox, (Octree.Cell.EChild)iChild));

                    cells.Add(childCell);

                    int numTris = cells[cellIndex].mTriangleIndices.Count;

                    for (int i = 0; i < numTris; i++)
                    {
                        int iTri = cells[cellIndex].mTriangleIndices[i];
                        IndexedTriangle tri = triangles[iTri];

                        if (DoesTriangleIntersectCell(tri, childCell))
                        {
                            childCell.mTriangleIndices.Add(iTri);
                        }
                    }
                }

                
                cells[cellIndex].mTriangleIndices.Clear();
            }
        }

        
        
        
        
        
        
        private AABox CreateAABox(AABox aabb, Octree.Cell.EChild child)
        {
            Vector3 dims = 0.5f * (aabb.MaxPos - aabb.MinPos);
            Vector3 offset = new Vector3();

            switch (child)
            {
                case Octree.Cell.EChild.PPP: offset = new Vector3(1, 1, 1); break;
                case Octree.Cell.EChild.PPM: offset = new Vector3(1, 1, 0); break;
                case Octree.Cell.EChild.PMP: offset = new Vector3(1, 0, 1); break;
                case Octree.Cell.EChild.PMM: offset = new Vector3(1, 0, 0); break;
                case Octree.Cell.EChild.MPP: offset = new Vector3(0, 1, 1); break;
                case Octree.Cell.EChild.MPM: offset = new Vector3(0, 1, 0); break;
                case Octree.Cell.EChild.MMP: offset = new Vector3(0, 0, 1); break;
                case Octree.Cell.EChild.MMM: offset = new Vector3(0, 0, 0); break;

                default:
                    System.Diagnostics.Debug.WriteLine("Octree.CreateAABox  got impossible child");
                    
                    
                    break;
            }

            AABox result = new AABox();
            result.MinPos = (aabb.MinPos + new Vector3(offset.X * dims.X, offset.Y * dims.Y, offset.Z * dims.Z));
            result.MaxPos = (result.MinPos + dims);

            
            float extra = 0.00001f;

            result.MinPos = (result.MinPos - extra * dims);
            result.MaxPos = (result.MaxPos + extra * dims);

            return result;
        }

        public int GetTrianglesIntersectingtAABox(List<int> triangles, AABox aabb)
        {
            if (cells.Count == 0)
                return 0;

            triangles.Clear();
            mCellsToTest.Clear();
            mCellsToTest.Push(0);

            IncrementTestCounter();

            while (mCellsToTest.Count != 0) 
            {
                int cellIndex = mCellsToTest.Pop();
                

                Octree.Cell cell = cells[cellIndex];

                if (!AABox.OverlapTest(aabb, cell.mAABox))
                    continue;

                if (cell.IsLeaf)
                {
                    int nTris = cell.mTriangleIndices.Count;

                    for (int i = 0; i < nTris; i++)
                    {
                        IndexedTriangle triangle = GetTriangle(cell.mTriangleIndices[i]);

                        if (triangle.counter != testCounter)
                        {
                            triangle.counter = testCounter;

                            if (AABox.OverlapTest(aabb, triangle.BoundingBox))
                                triangles.Add(cell.mTriangleIndices[i]);
                        }
                    }
                }
                else
                {
                    
                    for (int iChild = 0; iChild < Octree.NumChildren; iChild++)
                    {
                        int childIndex = cell.mChildCellIndices[iChild];
                        mCellsToTest.Push(childIndex);
                    }
                }
            }
            return triangles.Count;
        }

        private bool DoesTriangleIntersectCell(IndexedTriangle triangle, Octree.Cell cell)
        {
            if (!AABox.OverlapTest(triangle.BoundingBox, cell.mAABox))
                return false;

            
            if (cell.mAABox.IsPointInside(GetVertex(triangle.GetVertexIndex(0))) ||
                cell.mAABox.IsPointInside(GetVertex(triangle.GetVertexIndex(1))) ||
                cell.mAABox.IsPointInside(GetVertex(triangle.GetVertexIndex(2))))
                return true;

            
            
            Triangle tri =
 new Triangle(GetVertex(triangle.GetVertexIndex(0)), GetVertex(triangle.GetVertexIndex(1)), GetVertex(triangle.GetVertexIndex(2)));

            Box box = new Box(cell.mAABox.MinPos, Matrix.Identity, cell.mAABox.GetSideLengths());
            Vector3[] pts;

            box.GetCornerPoints(out pts);
            Box.Edge[] edges;
            box.GetEdges(out edges);

            for (int i = 0; i < 12; i++)
            {
                Box.Edge edge = edges[i];

                Segment seg = new Segment(pts[(int)edge.Ind0], pts[(int)edge.Ind1] - pts[(int)edge.Ind0]);

                if (Overlap.SegmentTriangleOverlap(seg, tri))
                    return true;
            }
            
            

            
            for (int iEdge = 0; iEdge < 3; ++iEdge)
            {
                Vector3 pt0 = tri.GetPoint(iEdge);
                Vector3 pt1 = tri.GetPoint((iEdge + 1) % 3);

                if (Overlap.SegmentAABoxOverlap(new Segment(pt0, pt1 - pt0), cell.mAABox))
                    return true;
            }

            return false;
        }

        
        
        
        
        private void IncrementTestCounter()
        {
            ++testCounter;

            if (testCounter == 0)
            {
                
                int numTriangles = triangles.Count;

                for (int i = 0; i < numTriangles; ++i)
                    triangles[i].counter = 0;

                testCounter = 1;
            }
        }

        
        
        
        
        
        public IndexedTriangle GetTriangle(int iTriangle)
        {
            return triangles[iTriangle];
        }

        
        
        
        
        
        public Vector3 GetVertex(int iVertex)
        {
            return vertices[iVertex];
        }

        public void GetVertex(int iVertex,out Vector3 result)
        {
            result = vertices[iVertex];
        }

        
        
        
        public int NumTriangles
        {
            get { return triangles.Count; }
        }



    }
#endif
}