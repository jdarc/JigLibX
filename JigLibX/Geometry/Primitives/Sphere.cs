using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Geometry.Primitives
{
    public class Sphere : Primitive
    {
        public Sphere(Vector3 pos, float radius) : base((int) PrimitiveType.Sphere)
        {
            transform.Position = pos;
            Radius = radius;
        }

        public override Primitive Clone()
        {
            return new Sphere(transform.Position, Radius);
        }

        public override bool SegmentIntersect(out float frac, out Vector3 pos, out Vector3 normal, Segment seg)
        {
            bool result;
            result = Intersection.SegmentSphereIntersection(out frac, seg, this);

            if (result)
            {
                pos = seg.GetPoint(frac);
                normal = pos - transform.Position;

                JiggleMath.NormalizeSafe(ref normal);
            }
            else
            {
                pos = Vector3.Zero;
                normal = Vector3.Zero;
            }

            return result;
        }

        public override void GetMassProperties(PrimitiveProperties primitiveProperties, out float mass, out Vector3 centerOfMass, out Matrix inertiaTensor)
        {
            if (primitiveProperties.MassType == PrimitiveProperties.MassTypeEnum.Mass)
            {
                mass = primitiveProperties.MassOrDensity;
            }
            else
            {
                if (primitiveProperties.MassDistribution == PrimitiveProperties.MassDistributionEnum.Solid)
                    mass = GetVolume() * primitiveProperties.MassOrDensity;
                else
                    mass = GetSurfaceArea() * primitiveProperties.MassOrDensity;
            }

            centerOfMass = transform.Position;
            float Ixx;
            if (primitiveProperties.MassDistribution == PrimitiveProperties.MassDistributionEnum.Solid)
                Ixx = 0.4f * mass * Radius;
            else
                Ixx = 2.0f / 3.0f * mass * Radius * Radius;


            inertiaTensor = Matrix.Identity;
            inertiaTensor.M11 = inertiaTensor.M22 = inertiaTensor.M33 = Ixx;


            inertiaTensor.M11 += mass * (centerOfMass.Y * centerOfMass.Y + centerOfMass.Z * centerOfMass.Z);
            inertiaTensor.M22 += mass * (centerOfMass.Z * centerOfMass.Z + centerOfMass.X * centerOfMass.X);
            inertiaTensor.M33 += mass * (centerOfMass.X * centerOfMass.X + centerOfMass.Y * centerOfMass.Y);

            inertiaTensor.M12 = inertiaTensor.M21 = inertiaTensor.M12 - mass * centerOfMass.X * centerOfMass.Y;
            inertiaTensor.M23 = inertiaTensor.M32 = inertiaTensor.M23 - mass * centerOfMass.Y * centerOfMass.Z;
            inertiaTensor.M31 = inertiaTensor.M13 = inertiaTensor.M31 - mass * centerOfMass.Z * centerOfMass.X;
        }

        public override Transform Transform
        {
            get => transform;
            set => transform = value;
        }

        public override float GetVolume()
        {
            return 4.0f / 3.0f * MathHelper.Pi * Radius * Radius * Radius;
        }

        public override float GetSurfaceArea()
        {
            return 4.0f * MathHelper.Pi * Radius * Radius;
        }

        public Vector3 Position
        {
            get => transform.Position;
            set => transform.Position = value;
        }

        public float Radius { get; set; }

        public static Sphere HugeSphere { get; } = new Sphere(Vector3.Zero, float.MaxValue);
    }
}