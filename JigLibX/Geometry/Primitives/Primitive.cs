using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Geometry.Primitives
{
    public abstract class Primitive
    {
        internal Transform transform = Transform.Identity;

        public Primitive(int type)
        {
            Type = type;
        }

        public abstract Primitive Clone();

        public virtual Transform Transform
        {
            get => transform;
            set => transform = value;
        }

        public virtual Matrix TransformMatrix
        {
            get
            {
                var trans = transform.Orientation;
                trans.Translation = transform.Position;
                return trans;
            }
        }

        public virtual Matrix InverseTransformMatrix
        {
            get
            {
                var trans = transform.Orientation;
                trans.Translation = transform.Position;
                return Matrix.Invert(trans);
            }
        }

        public abstract bool SegmentIntersect(out float frac, out Vector3 pos, out Vector3 normal, Segment seg);

        public abstract float GetVolume();

        public abstract float GetSurfaceArea();

        public abstract void GetMassProperties(PrimitiveProperties primitiveProperties, out float mass, out Vector3 centerOfMass, out Matrix inertiaTensor);

        public virtual void GetBoundingBox(out AABox box)
        {
            box = AABox.HugeBox;
        }

        public AABox GetBoundingBox()
        {
            GetBoundingBox(out var result);
            return result;
        }

        public int Type { get; }
    }
}