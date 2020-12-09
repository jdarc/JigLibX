using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Geometry.Primitives
{
    public class Plane : Primitive
    {
        internal Vector3 normal = Vector3.Zero;

        public Plane() : base((int) PrimitiveType.Plane)
        {
        }

        public Plane(Vector3 n, float d) : base((int) PrimitiveType.Plane)
        {
            JiggleMath.NormalizeSafe(ref n);
            normal = n;
            D = d;
        }

        public Plane(Vector3 n, Vector3 pos) : base((int) PrimitiveType.Plane)
        {
            JiggleMath.NormalizeSafe(ref n);
            normal = n;
            D = -Vector3.Dot(n, pos);
        }

        public Plane(Vector3 pos0, Vector3 pos1, Vector3 pos2) : base((int) PrimitiveType.Plane)
        {
            var dr1 = pos1 - pos0;
            var dr2 = pos2 - pos0;

            normal = Vector3.Cross(dr1, dr2);
            var mNLen = normal.Length();
            if (mNLen < JiggleMath.Epsilon)
            {
                normal = Vector3.Up;
                D = 0.0f;
            }
            else
            {
                normal /= mNLen;
                D = -Vector3.Dot(normal, pos0);
            }
        }

        public Vector3 Normal
        {
            get => normal;
            set => normal = value;
        }

        public float D { get; set; }

        public override Primitive Clone()
        {
            var newPlane = new Plane(Normal, D);
            newPlane.Transform = Transform;
            return newPlane;
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

        public override bool SegmentIntersect(out float frac, out Vector3 pos, out Vector3 normal, Segment seg)
        {
            bool result;
            if (result = Intersection.SegmentPlaneIntersection(out frac, seg, this))
            {
                pos = seg.GetPoint(frac);
                normal = Normal;
            }
            else
            {
                pos = Vector3.Zero;
                normal = Vector3.Zero;
            }

            return result;
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

        public void Invert()
        {
            Vector3.Negate(ref normal, out normal);
        }

        public Plane GetInverse()
        {
            var plane = new Plane(normal, D);
            plane.Invert();
            return plane;
        }
    }
}