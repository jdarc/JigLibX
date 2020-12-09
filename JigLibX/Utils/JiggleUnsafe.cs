using Microsoft.Xna.Framework;

namespace JigLibX.Utils
{
    public static class JiggleUnsafe
    {
        public static unsafe float Get(ref Vector3 vec, int index)
        {
            fixed (Vector3* adr = &vec) return ((float*) adr)[index];
        }

        public static unsafe float Get(Vector3 vec, int index) => ((float*) &vec)[index];

        public static unsafe Vector3 Get(Matrix mat, int index)
        {
            var adr = &mat.M11;
            adr += index;
            return ((Vector3*) adr)[index];
        }

        public static unsafe void Get(ref Matrix mat, int index, out Vector3 vec)
        {
            fixed (float* adr = &mat.M11)
            {
                vec = ((Vector3*) (adr + index))[index];
            }
        }
    }
}