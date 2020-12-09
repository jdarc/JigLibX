using Microsoft.Xna.Framework;

namespace JigLibX.Math
{
    public struct Transform
    {
        public Vector3 Position;

        public Matrix Orientation;

        public static Transform Identity => new Transform(Vector3.Zero, Matrix.Identity);

        public Transform(Vector3 position, Matrix orientation)
        {
            Position = position;
            Orientation = orientation;
        }

        public void ApplyTransformRate(ref TransformRate rate, float dt)
        {
            Vector3.Multiply(ref rate.Velocity, dt, out var pos);
            Vector3.Add(ref Position, ref pos, out Position);

            var dir = rate.AngularVelocity;
            var ang = dir.Length();

            if (ang > 0.0f)
            {
                Vector3.Divide(ref dir, ang, out dir);
                ang *= dt;
                Matrix.CreateFromAxisAngle(ref dir, ang, out var rot);
                Matrix.Multiply(ref Orientation, ref rot, out Orientation);
            }
        }

        public void ApplyTransformRate(TransformRate rate, float dt)
        {
            Vector3.Multiply(ref rate.Velocity, dt, out var pos);
            Vector3.Add(ref Position, ref pos, out Position);

            var dir = rate.AngularVelocity;
            var ang = dir.Length();

            if (ang > 0.0f)
            {
                Vector3.Divide(ref dir, ang, out dir);
                ang *= dt;
                Matrix.CreateFromAxisAngle(ref dir, ang, out var rot);
                Matrix.Multiply(ref Orientation, ref rot, out Orientation);
            }
        }

        public static Transform operator *(Transform lhs, Transform rhs)
        {
            Multiply(ref lhs, ref rhs, out var result);
            return result;
        }

        public static Transform Multiply(Transform lhs, Transform rhs)
        {
            var result = new Transform();
            Matrix.Multiply(ref rhs.Orientation, ref lhs.Orientation, out result.Orientation);

            Vector3.TransformNormal(ref rhs.Position, ref lhs.Orientation, out result.Position);
            Vector3.Add(ref lhs.Position, ref result.Position, out result.Position);


            return result;
        }

        public static void Multiply(ref Transform lhs, ref Transform rhs, out Transform result)
        {
            result = new Transform();

            Matrix.Multiply(ref rhs.Orientation, ref lhs.Orientation, out result.Orientation);

            Vector3.TransformNormal(ref rhs.Position, ref lhs.Orientation, out result.Position);
            Vector3.Add(ref lhs.Position, ref result.Position, out result.Position);
        }
    }
}