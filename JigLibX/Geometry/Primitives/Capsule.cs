using JigLibX.Math;
using Microsoft.Xna.Framework;

namespace JigLibX.Geometry.Primitives
{
    public class Capsule : Primitive
    {
        public Capsule(Vector3 pos, Matrix orient, float radius, float length) : base((int) PrimitiveType.Capsule)
        {
            transform = new Transform(pos, orient);
            Length = length;
            Radius = radius;
        }

        public override bool SegmentIntersect(out float frac, out Vector3 pos, out Vector3 normal, Segment seg)
        {
            var result = Intersection.SegmentCapsuleIntersection(out frac, seg, this);

            if (result)
            {
                pos = seg.GetPoint(frac);
                normal = pos - transform.Position;
                normal -= Vector3.Dot(normal, transform.Orientation.Backward) * transform.Orientation.Backward;
                JiggleMath.NormalizeSafe(ref normal);
            }
            else
            {
                pos = normal = Vector3.Zero;
            }

            return result;
        }

        public override Primitive Clone()
        {
            return new Capsule(transform.Position, transform.Orientation, Radius, Length);
        }

        public override Transform Transform
        {
            get => transform;
            set => transform = value;
        }

        public Vector3 Position
        {
            get => transform.Position;
            set => transform.Position = value;
        }

        public Vector3 GetEnd()
        {
            return transform.Position + Length * transform.Orientation.Backward;
        }

        public Matrix Orientation
        {
            get => transform.Orientation;
            set => transform.Orientation = value;
        }

        public float Length { get; set; }

        public float Radius { get; set; }

        public override float GetVolume()
        {
            return 4.0f / 3.0f * MathHelper.Pi * Radius * Radius * Radius + Length * MathHelper.Pi * Radius * Radius;
        }

        public override float GetSurfaceArea()
        {
            return 4.0f * MathHelper.Pi * Radius * Radius + Length * 2.0f * MathHelper.Pi * Radius;
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

            centerOfMass = transform.Position + 0.5f * Length * transform.Orientation.Backward;


            var cylinderMass = mass * MathHelper.Pi * Radius * Radius * Length / GetVolume();
            var Ixx = 0.5f * cylinderMass * Radius * Radius;
            var Iyy = 0.25f * cylinderMass * Radius * Radius + 1.0f / 12.0f * cylinderMass * Length * Length;
            var Izz = Iyy;

            var endMass = mass - cylinderMass;
            Ixx += 0.4f * endMass * Radius * Radius;
            Iyy += 0.4f * endMass * Radius * Radius + endMass * (0.5f * Length) * (0.5f * Length);
            Izz += 0.4f * endMass * Radius * Radius + endMass * (0.5f * Length) * (0.5f * Length);

            inertiaTensor = Matrix.Identity;
            inertiaTensor.M11 = Ixx;
            inertiaTensor.M22 = Iyy;
            inertiaTensor.M33 = Izz;


            inertiaTensor = transform.Orientation * inertiaTensor * Matrix.Transpose(transform.Orientation);


            inertiaTensor.M11 += mass * (centerOfMass.Y * centerOfMass.Y + centerOfMass.Z * centerOfMass.Z);
            inertiaTensor.M22 += mass * (centerOfMass.Z * centerOfMass.Z + centerOfMass.X * centerOfMass.X);
            inertiaTensor.M33 += mass * (centerOfMass.X * centerOfMass.X + centerOfMass.Y * centerOfMass.Y);

            inertiaTensor.M12 = inertiaTensor.M21 = inertiaTensor.M12 - mass * centerOfMass.X * centerOfMass.Y;
            inertiaTensor.M23 = inertiaTensor.M32 = inertiaTensor.M23 - mass * centerOfMass.Y * centerOfMass.Z;
            inertiaTensor.M31 = inertiaTensor.M13 = inertiaTensor.M31 - mass * centerOfMass.Z * centerOfMass.X;
        }
    }
}