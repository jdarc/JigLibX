using Microsoft.Xna.Framework;
using JigLibX.Math;

namespace JigLibX.Geometry
{
    public struct Triangle
    {
        private Vector3 origin;
        private Vector3 edge0;
        private Vector3 edge1;

        public Triangle(Vector3 pt0, Vector3 pt1, Vector3 pt2)
        {
            origin = pt0;
            edge0 = pt1 - pt0;
            edge1 = pt2 - pt0;
        }

        public Triangle(ref Vector3 pt0, ref Vector3 pt1, ref Vector3 pt2)
        {
            origin = pt0;
            edge0 = pt1 - pt0;
            edge1 = pt2 - pt0;
        }

        public Vector3 GetPoint(int i)
        {
            if (i == 1) return origin + edge0;

            if (i == 2) return origin + edge1;

            return origin;
        }

        public void GetPoint(int i, out Vector3 point)
        {
            if (i == 1)
                point = origin + edge0;
            else if (i == 2)
                point = origin + edge1;
            else
                point = origin;
        }

        public void GetPoint(ref Vector3 point, int i)
        {
            if (i == 1)
            {
                point.X = origin.X + edge0.X;
                point.Y = origin.Y + edge0.Y;
                point.Z = origin.Z + edge0.Z;
            }
            else if (i == 2)
            {
                point.X = origin.X + edge1.X;
                point.Y = origin.Y + edge1.Y;
                point.Z = origin.Z + edge1.Z;
            }
            else
            {
                point.X = origin.X;
                point.Y = origin.Y;
                point.Z = origin.Z;
            }
        }

        public Vector3 GetPoint(float t0, float t1)
        {
            return origin + t0 * edge0 + t1 * edge1;
        }

        public void GetSpan(out float min, out float max, Vector3 axis)
        {
            var d0 = Vector3.Dot(GetPoint(0), axis);
            var d1 = Vector3.Dot(GetPoint(1), axis);
            var d2 = Vector3.Dot(GetPoint(2), axis);

            min = JiggleMath.Min(d0, d1, d2);
            max = JiggleMath.Max(d0, d1, d2);
        }

        public void GetSpan(out float min, out float max, ref Vector3 axis)
        {
            var point = new Vector3();

            GetPoint(ref point, 0);
            var d0 = point.X * axis.X + point.Y * axis.Y + point.Z * axis.Z;
            GetPoint(ref point, 1);
            var d1 = point.X * axis.X + point.Y * axis.Y + point.Z * axis.Z;
            GetPoint(ref point, 2);
            var d2 = point.X * axis.X + point.Y * axis.Y + point.Z * axis.Z;

            min = JiggleMath.Min(d0, d1, d2);
            max = JiggleMath.Max(d0, d1, d2);
        }

        public Vector3 Centre => origin + 0.333333333333f * (edge0 + edge1);

        public Vector3 Origin
        {
            get => origin;
            set => origin = value;
        }

        public Vector3 Edge0
        {
            get => edge0;
            set => edge0 = value;
        }

        public Vector3 Edge1
        {
            get => edge1;
            set => edge1 = value;
        }

        public Vector3 Edge2 => edge1 - edge0;

        public Plane Plane => new Plane(GetPoint(0), GetPoint(1), GetPoint(2));

        public Vector3 Normal
        {
            get
            {
                var norm = Vector3.Cross(edge0, edge1);
                JiggleMath.NormalizeSafe(ref norm);

                return norm;
            }
        }
    }
}