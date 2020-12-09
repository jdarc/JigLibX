using System;
using System.Linq;
using Microsoft.Xna.Framework;

namespace JigLibX.Utils
{
    public class Array2D
    {
        public delegate float Function(int x, int z, Array2D arrInstance);

        public float[] Array;

        public Array2D(Array2D arr)
        {
            if (arr?.Array == null) return;

            Array = new float[arr.Array.Length];
            Nx = arr.Nx;
            Nz = arr.Nz;

            Buffer.BlockCopy(arr.Array, 0, Array, 0, Array.Length * 4);
        }

        public Array2D(int nx, int nz)
        {
            Array = new float[nx * nz];
            Nx = nx;
            Nz = nz;
        }

        public Array2D(int nx, int nz, float val)
        {
            Array = new float[nx * nz];
            Nx = nx;
            Nz = nz;

            for (var i = 0; i < Array.Length; i++) Array[i] = val;
        }

        public static Array2D CreateArray(int nx, int nz, Function func)
        {
            var arr = new Array2D(nx, nz);

            if (func == null) return arr;

            for (var xx = 0; xx < nx; xx++)
            for (var zz = 0; zz < nz; zz++)
                arr.Array[xx + zz * nx] = func(xx, zz, arr);

            return arr;
        }

        public void Resize(int nx, int nz)
        {
            if (nx == Nx && nz == Nz) return;

            Array = new float[nx * nz];

            Nx = nx;
            Nz = nz;
        }

        public Array2D Pow(float rhs)
        {
            for (var i = 0; i < Nx * Nz; ++i) Array[i] = (float) System.Math.Pow(Array[i], rhs);
            return this;
        }

        public void Abs()
        {
            for (var i = 0; i < Array.Length; ++i)
                if (Array[i] < 0)
                    Array[i] = -Array[i];
        }

        public void GaussianFilter(float r, int n)
        {
            int i;
            int j;

            var size = n * 2 + 1;
            var filter = new float[size * size];

            for (i = 0; i < size; ++i)
            for (j = 0; j < size; ++j)
                filter[i + j * size] = (float) System.Math.Exp(-((i - n) * (i - n) + (j - n) * (j - n)) / (r * r));

            for (i = 0; i < Nx; ++i)
            for (j = 0; j < Nz; ++j)
            {
                float total = 0;
                float weight_total = 0;

                int ii;
                for (ii = -n; ii < n; ++ii)
                {
                    int iii;
                    if ((iii = i + ii) >= 0 && iii < Nx || Wrap)
                    {
                        int jj;
                        for (jj = -n; jj < n; ++jj)
                        {
                            int jjj;
                            if ((jjj = j + jj) >= 0 && jjj < Nz || Wrap)
                            {
                                var index = n + ii + (n + jj) * size;

                                weight_total += filter[index];
                                total += filter[index] * GetAt(iii, jjj);
                            }
                        }
                    }
                }

                SetAt(i, j, total / weight_total);
            }
        }

        public void Shift(int offsetX, int offsetZ)
        {
            var orig = new Array2D(this);

            for (var i = 0; i < Nx; ++i)
            for (var j = 0; j < Nz; ++j)
            {
                var i0 = (i + offsetX) % Nx;
                var j0 = (j + offsetZ) % Nz;

                SetAt(i0, j0, orig.GetAt(i, j));
            }
        }

        public void SetRange(float valMin, float valMax)
        {
            int i;
            var origMin = Min;
            var origMax = Max;
            var scale = (valMax - valMin) / (origMax - origMin);
            var offset = valMin - scale * origMin;
            for (i = 0; i < Array.Length; ++i) Array[i] = scale * Array[i] + offset;
        }

        public void SetTo(float val)
        {
            for (var i = 0; i < Array.Length; ++i) Array[i] = val;
        }

        public float Interpolate(float fi, float fj)
        {
            fi = MathHelper.Clamp(fi, 0.0f, Nx - 1.0f);
            fj = MathHelper.Clamp(fj, 0.0f, Nz - 1.0f);

            var i0 = (int) fi;
            var j0 = (int) fj;
            var i1 = i0 + 1;
            var j1 = j0 + 1;

            if (i1 >= Nx) i1 = Nx - 1;
            if (j1 >= Nz) j1 = Nz - 1;

            var iFrac = fi - i0;
            var jFrac = fj - j0;

            var result = jFrac * (iFrac * this[i1, j1] + (1.0f - iFrac) * this[i0, j1]) + (1.0f - jFrac) * (iFrac * this[i1, j0] + (1.0f - iFrac) * this[i0, j0]);

            return result;
        }

        public float GetAt(int i, int j)
        {
            UnwrapIndices(ref i, ref j);
            return Array[i + j * Nx];
        }

        public void SetAt(int i, int j, float val)
        {
            UnwrapIndices(ref i, ref j);
            Array[i + j * Nx] = val;
        }

        private void UnwrapIndices(ref int i, ref int j)
        {
            if (Wrap == false) return;
            while (i < 0) i += Nx;
            while (j < 0) j += Nz;
            i %= Nx;
            j %= Nz;
        }

        public float this[int i, int j] => Array[i + j * Nx];

        public bool Wrap { get; set; }

        public int Nx { get; private set; }
        public int Nz { get; private set; }

        public float Min => Array.Prepend(Array[0]).Min();
        public float Max => Array.Prepend(Array[0]).Max();
    }
}