using Microsoft.Xna.Framework;

namespace JigLibX.Math
{
    public static class JiggleMath
    {
        public const float Epsilon = 1.0e-6f;

        public static float Wrap(float val, float min, float max)
        {
            var delta = max - min;
            if (val > delta)
            {
                val /= delta;
                val -= (float) System.Math.Floor(val);
                val *= delta;
            }

            return val;
        }

        public static float SafeInvScalar(float val)
        {
            if (System.Math.Abs(val) < Epsilon) return float.MaxValue;
            return 1.0f / val;
        }

        public static void NormalizeSafe(ref Vector3 vec)
        {
            var num0 = vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z;

            if (num0 != 0.0f)
            {
                var num1 = 1.0f / (float) System.Math.Sqrt(num0);
                vec.X *= num1;
                vec.Y *= num1;
                vec.Z *= num1;
            }
        }

        public static Vector3 NormalizeSafe(Vector3 vec)
        {
            var num0 = vec.X * vec.X + vec.Y * vec.Y + vec.Z * vec.Z;

            if (num0 != 0.0f)
            {
                var num1 = 1.0f / (float) System.Math.Sqrt(num0);
                vec.X *= num1;
                vec.Y *= num1;
                vec.Z *= num1;
            }

            return vec;
        }

        public static void Orthonormalise(ref Matrix matrix)
        {
            var u11 = matrix.M11;
            var u12 = matrix.M12;
            var u13 = matrix.M13;
            var u21 = matrix.M21;
            var u22 = matrix.M22;
            var u23 = matrix.M23;
            var u31 = matrix.M31;
            var u32 = matrix.M32;
            var u33 = matrix.M33;

            float dot0, dot1;


            var lengthSq0 = u11 * u11 + u12 * u12 + u13 * u13;
            var length0 = (float) System.Math.Sqrt(lengthSq0);
            u11 /= length0;
            u12 /= length0;
            u13 /= length0;


            dot0 = u11 * u21 + u12 * u22 + u13 * u23;
            u21 -= dot0 * u11 / lengthSq0;
            u22 -= dot0 * u12 / lengthSq0;
            u23 -= dot0 * u13 / lengthSq0;

            var lengthSq1 = u21 * u21 + u22 * u22 + u23 * u23;
            var length1 = (float) System.Math.Sqrt(lengthSq1);
            u21 /= length1;
            u22 /= length1;
            u23 /= length1;


            dot0 = u11 * u31 + u12 * u32 + u13 * u33;
            dot1 = u21 * u31 + u22 * u32 + u23 * u33;
            u31 = u31 - dot0 * u11 / lengthSq0 - dot1 * u21 / lengthSq1;
            u32 = u32 - dot0 * u12 / lengthSq0 - dot1 * u22 / lengthSq1;
            u33 = u33 - dot0 * u13 / lengthSq0 - dot1 * u23 / lengthSq1;

            lengthSq0 = u31 * u31 + u32 * u32 + u33 * u33;
            length0 = (float) System.Math.Sqrt(lengthSq0);
            u31 /= length0;
            u32 /= length0;
            u33 /= length0;

            matrix.M11 = u11;
            matrix.M12 = u12;
            matrix.M13 = u13;
            matrix.M21 = u21;
            matrix.M22 = u22;
            matrix.M23 = u23;
            matrix.M31 = u31;
            matrix.M32 = u32;
            matrix.M33 = u33;
        }

        public static Matrix RotationMatrix(float ang, Vector3 dir)
        {
            return Matrix.CreateFromAxisAngle(dir, MathHelper.ToRadians(ang));
        }

        public static float Max(float a, float b, float c)
        {
            var abMax = a > b ? a : b;

            return abMax > c ? abMax : c;
        }

        public static float Min(float a, float b, float c)
        {
            var abMin = a < b ? a : b;

            return abMin < c ? abMin : c;
        }
    }
}