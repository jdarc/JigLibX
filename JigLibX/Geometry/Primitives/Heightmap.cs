using JigLibX.Math;
using JigLibX.Utils;
using Microsoft.Xna.Framework;

namespace JigLibX.Geometry.Primitives
{
    public class Heightmap : Primitive
    {
        private float x0, z0;
        private float dx, dz;
        private float xMin, zMin;
        private float xMax, zMax;
        private float yMax, yMin;

        public Vector3 Min => new Vector3(xMin, yMin, zMin);

        public Vector3 Max => new Vector3(xMax, yMax, zMax);

        public Heightmap(Array2D heights, float x0, float z0, float dx, float dz) : base((int) PrimitiveType.Heightmap)
        {
            Heights = heights;
            this.x0 = x0;
            this.z0 = z0;
            this.dx = dx;
            this.dz = dz;

            xMin = x0 - (Heights.Nx - 1) * 0.5f * dx;
            zMin = z0 - (Heights.Nz - 1) * 0.5f * dz;
            xMax = x0 + (Heights.Nx - 1) * 0.5f * dx;
            zMax = z0 + (Heights.Nz - 1) * 0.5f * dz;


            yMin = Heights.Min;
            yMax = Heights.Max;
        }

        public void RecalculateBoundingBox()
        {
            xMin = x0 - (Heights.Nx - 1) * 0.5f * dx;
            zMin = z0 - (Heights.Nz - 1) * 0.5f * dz;
            xMax = x0 + (Heights.Nx - 1) * 0.5f * dx;
            zMax = z0 + (Heights.Nz - 1) * 0.5f * dz;
            yMin = Heights.Min;
            yMax = Heights.Max;
        }

        public override void GetBoundingBox(out AABox box)
        {
            box = new AABox(Min, Max);
        }

        public float GetHeight(int i, int j)
        {
            i = MathHelper.Clamp(i, 0, Heights.Nx - 1);
            j = MathHelper.Clamp(j, 0, Heights.Nz - 1);

            return Heights[i, j];
        }

        public Vector3 GetNormal(int i, int j)
        {
            var i0 = i - 1;
            var i1 = i + 1;
            var j0 = j - 1;
            var j1 = j + 1;
            i0 = MathHelper.Clamp(i0, 0, Heights.Nx - 1);
            j0 = MathHelper.Clamp(j0, 0, Heights.Nz - 1);
            i1 = MathHelper.Clamp(i1, 0, Heights.Nx - 1);
            j1 = MathHelper.Clamp(j1, 0, Heights.Nz - 1);

            var dx = (i1 - i0) * this.dx;
            var dz = (j1 - j0) * this.dz;

            if (i0 == i1) dx = 1.0f;
            if (j0 == j1) dz = 1.0f;

            if (i0 == i1 && j0 == j1) return Vector3.Up;

            var hFwd = Heights[i1, j];
            var hBack = Heights[i0, j];
            var hLeft = Heights[i, j1];
            var hRight = Heights[i, j0];

            var v1 = new Vector3(dx, hFwd - hBack, 0.0f);
            var v2 = new Vector3(0.0f, hLeft - hRight, dz);

            Vector3.Cross(ref v1, ref v2, out var normal);
            normal.Normalize();

            return normal;
        }

        public void GetHeightAndNormal(out float h, out Vector3 normal, int i, int j)
        {
            h = GetHeight(i, j);
            normal = GetNormal(i, j);
        }

        public void GetSurfacePos(out Vector3 pos, int i, int j)
        {
            var h = GetHeight(i, j);
            pos = new Vector3(xMin + i * dx, h, zMin + j * dz);
        }

        public void GetSurfacePosAndNormal(out Vector3 pos, out Vector3 normal, int i, int j)
        {
            var h = GetHeight(i, j);
            pos = new Vector3(xMin + i * dx, h, zMin + j * dz);
            normal = GetNormal(i, j);
        }

        public float GetHeight(Vector3 point)
        {
            GetHeightAndNormal(out var h, out var normal, point);
            return h;
        }

        public Vector3 GetNormal(Vector3 point)
        {
            GetHeightAndNormal(out var h, out var normal, point);
            return normal;
        }

        public void GetHeightAndNormal(out float h, out Vector3 normal, Vector3 point)
        {
            var x = point.X;
            var z = point.Z;

            x = MathHelper.Clamp(x, xMin, xMax);
            z = MathHelper.Clamp(z, zMin, zMax);

            var i0 = (int) ((x - xMin) / dx);
            var j0 = (int) ((point.Z - zMin) / dz);

            i0 = MathHelper.Clamp(i0, 0, Heights.Nx - 1);
            j0 = MathHelper.Clamp(j0, 0, Heights.Nz - 1);

            var i1 = i0 + 1;
            var j1 = j0 + 1;

            if (i1 >= Heights.Nx) i1 = Heights.Nx - 1;
            if (j1 >= Heights.Nz) j1 = Heights.Nz - 1;

            var iFrac = (x - (i0 * dx + xMin)) / dx;
            var jFrac = (z - (j0 * dz + zMin)) / dz;

            iFrac = MathHelper.Clamp(iFrac, 0.0f, 1.0f);
            jFrac = MathHelper.Clamp(jFrac, 0.0f, 1.0f);

            var h00 = Heights[i0, j0];
            var h01 = Heights[i0, j1];
            var h10 = Heights[i1, j0];
            var h11 = Heights[i1, j1];


            if (i0 == i1 && j0 == j1)
            {
                normal = Vector3.Up;
            }
            else if (i0 == i1)
            {
                var right = Vector3.Right;
                normal = Vector3.Cross(new Vector3(0.0f, h01 - h00, dz), right);
                normal.Normalize();
            }

            if (j0 == j1)
            {
                var backw = Vector3.Backward;
                normal = Vector3.Cross(backw, new Vector3(dx, h10 - h00, 0.0f));
                normal.Normalize();
            }
            else if (iFrac > jFrac)
            {
                normal = Vector3.Cross(new Vector3(dx, h11 - h00, dz), new Vector3(dx, h10 - h00, 0.0f));
                normal.Normalize();
            }
            else
            {
                normal = Vector3.Cross(new Vector3(0.0f, h01 - h00, dz), new Vector3(dx, h11 - h00, dz));
                normal.Normalize();
            }


            JiggleMath.NormalizeSafe(ref normal);
            var pos = new Vector3(i0 * dx + xMin, h00, j0 * dz + zMin);
            Vector3.Dot(ref normal, ref pos, out var d);
            d = -d;
            h = Distance.PointPlaneDistance(ref point, ref normal, d);
        }

        public void GetSurfacePos(out Vector3 pos, Vector3 point)
        {
            var h = GetHeight(point);
            pos = new Vector3(point.X, h, point.Z);
        }

        public void GetSurfacePosAndNormal(out Vector3 pos, out Vector3 normal, Vector3 point)
        {
            GetHeightAndNormal(out var h, out normal, point);
            pos = new Vector3(point.X, h, point.Z);
        }

        public override Primitive Clone()
        {
            return new Heightmap(new Array2D(Heights), x0, z0, dx, dz);
        }

        public override Transform Transform
        {
            get => Transform.Identity;
            set { }
        }

        public override bool SegmentIntersect(out float frac, out Vector3 pos, out Vector3 normal, Segment seg)
        {
            frac = 0;
            pos = Vector3.Zero;
            normal = Vector3.Up;


            GetHeightAndNormal(out var heightStart, out var normalStart, seg.Origin);

            if (heightStart < 0.0f) return false;

            var end = seg.GetEnd();
            GetHeightAndNormal(out var heightEnd, out var normalEnd, end);

            if (heightEnd > 0.0f) return false;


            var depthEnd = -heightEnd;


            var weightStart = 1.0f / (JiggleMath.Epsilon + heightStart);
            var weightEnd = 1.0f / (JiggleMath.Epsilon + depthEnd);

            normal = (normalStart * weightStart + normalEnd * weightEnd) / (weightStart + weightEnd);

            frac = heightStart / (heightStart + depthEnd + JiggleMath.Epsilon);

            pos = seg.GetPoint(frac);

            return true;
        }

        public override float GetVolume()
        {
            return 0.0f;
        }

        public override float GetSurfaceArea()
        {
            return 0.0f;
        }

        public override void GetMassProperties(PrimitiveProperties primitiveProperties, out float mass, out Vector3 centerOfMass, out Matrix inertiaTensor)
        {
            mass = 0.0f;
            centerOfMass = Vector3.Zero;
            inertiaTensor = Matrix.Identity;
        }

        public Array2D Heights { get; }
    }
}