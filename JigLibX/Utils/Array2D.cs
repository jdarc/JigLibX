﻿using System;
using Microsoft.Xna.Framework;

namespace JigLibX.Utils {
    /// <summary>
    /// Defines a 2D Array
    /// </summary>
    public class Array2D {
        /// <summary>
        /// Delegate function
        /// </summary>
        /// <param name="x"></param>
        /// <param name="z"></param>
        /// <param name="arrInstance"></param>
        /// <returns>float</returns>
        public delegate float Function(int x, int z, Array2D arrInstance);

        private int nx, nz;
        private bool wrap;

        /// <summary>
        /// Array
        /// </summary>
        public float[] Array;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="arr"></param>
        public Array2D(Array2D arr) {
            if (arr == null || arr.Array == null) return;

            Array = new float[arr.Array.Length];
            nx = arr.Nx;
            nz = arr.Nz;

            Buffer.BlockCopy(arr.Array, 0, Array, 0, Array.Length * 4);
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="nx"></param>
        /// <param name="nz"></param>
        public Array2D(int nx, int nz) {
            Array = new float[nx * nz];
            this.nx = nx;
            this.nz = nz;
        }

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="nx"></param>
        /// <param name="nz"></param>
        /// <param name="val"></param>
        public Array2D(int nx, int nz, float val) {
            Array = new float[nx * nz];
            this.nx = nx;
            this.nz = nz;

            for (int i = 0; i < Array.Length; i++) Array[i] = val;
        }

        /// <summary>
        /// Creates 2D array from mathematical function
        /// </summary>
        /// <param name="nx"></param>
        /// <param name="nz"></param>
        /// <param name="func"></param>
        /// <returns>Array2D</returns>
        public static Array2D CreateArray(int nx, int nz, Function func) {
            Array2D arr = new Array2D(nx, nz);

            if (func == null) return arr;

            for (int xx = 0; xx < nx; xx++)
            for (int zz = 0; zz < nz; zz++)
                arr.Array[xx + zz * nx] = func(xx, zz, arr);

            return arr;
        }

        /// <summary>
        /// Allows resizing. Data will be lost if resizing occurred
        /// </summary>
        /// <param name="nx"></param>
        /// <param name="nz"></param>
        public void Resize(int nx, int nz) {
            if (nx == this.nx && nz == this.nz) return;

            Array = new float[nx * nz];

            this.nx = nx;
            this.nz = nz;
        }

        /// <summary>
        /// Raise array elements to a power
        /// </summary>
        /// <param name="rhs"></param>
        /// <returns>Array2D</returns>
        public Array2D Pow(float rhs) {
            for (int i = 0; i < nx * nz; ++i) Array[i] = (float) System.Math.Pow(Array[i], rhs);

            return this;
        }

        /// <summary>
        /// Sets each value to its absolute value by comparison with float(0)
        /// </summary>
        public void Abs() {
            for (int i = 0; i < Array.Length; ++i)
                if (Array[i] < 0)
                    Array[i] = -Array[i];
        }

        /// <summary>
        /// Apply a Gaussian filter with length scale r, extending over a
        /// square of half-width n (so n=1 uses a square of 9 points, n = 2
        /// uses 25 etc). Suggest using n at least 2*r.
        /// </summary>
        /// <param name="r">length scale</param>
        /// <param name="n">half-width</param>
        public void GaussianFilter(float r, int n) {
            int i, j, ii, jj, iii, jjj;

            int size = n * 2 + 1;
            float[] filter = new float[size * size];

            for (i = 0; i < size; ++i)
            for (j = 0; j < size; ++j)
                filter[i + j * size] = (float) System.Math.Exp(-((i - n) * (i - n) + (j - n) * (j - n)) / (r * r));

            for (i = 0; i < (int) nx; ++i)
            for (j = 0; j < (int) nz; ++j) {
                float total = 0;
                float weight_total = 0;

                for (ii = -n; ii < (int) n; ++ii)
                    if ((iii = i + ii) >= 0 && iii < nx || wrap)
                        for (jj = -n; jj < (int) n; ++jj)
                            if ((jjj = j + jj) >= 0 && jjj < nz || wrap) {
                                // in a valid location
                                int index = n + ii + (n + jj) * size;

                                weight_total += filter[index];
                                total += filter[index] * GetAt(iii, jjj);
                            }

                SetAt(i, j, total / weight_total);
            }
        }

        /// <summary>
        /// Shifts all the elements...
        /// </summary>
        /// <param name="offsetX"></param>
        /// <param name="offsetZ"></param>
        public void Shift(int offsetX, int offsetZ) {
            Array2D orig = new Array2D(this);

            for (int i = 0; i < nx; ++i)
            for (int j = 0; j < nz; ++j) {
                int i0 = (i + offsetX) % nx;
                int j0 = (j + offsetZ) % nz;

                SetAt(i0, j0, orig.GetAt(i, j));
            }
        }

        /// <summary>
        /// Scale to fit within range...
        /// </summary>
        /// <param name="valMin"></param>
        /// <param name="valMax"></param>
        public void SetRange(float valMin, float valMax) {
            int i;
            float origMin = Min;
            float origMax = Max;

            // set min to 0 and scale...
            float scale = (valMax - valMin) / (origMax - origMin);
            float offset = valMin - scale * origMin;

            for (i = 0; i < Array.Length; ++i) Array[i] = scale * Array[i] + offset;
        }

        /// <summary>
        /// Set to a constant value
        /// </summary>
        /// <param name="val"></param>
        public void SetTo(float val) {
            for (int i = 0; i < Array.Length; ++i) Array[i] = val;
        }

        /// <summary>
        /// Interpolate
        /// </summary>
        /// <param name="fi"></param>
        /// <param name="fj"></param>
        /// <returns>float</returns>
        public float Interpolate(float fi, float fj) {
            fi = MathHelper.Clamp(fi, 0.0f, nx - 1.0f);
            fj = MathHelper.Clamp(fj, 0.0f, nz - 1.0f);

            int i0 = (int) fi;
            int j0 = (int) fj;
            int i1 = i0 + 1;
            int j1 = j0 + 1;

            if (i1 >= nx) i1 = nx - 1;
            if (j1 >= nz) j1 = nz - 1;

            float iFrac = fi - i0;
            float jFrac = fj - j0;

            float result = jFrac * (iFrac * this[i1, j1] + (1.0f - iFrac) * this[i0, j1]) + (1.0f - jFrac) * (iFrac * this[i1, j0] + (1.0f - iFrac) * this[i0, j0]);

            return result;
        }

        /// <summary>
        /// Checked access - unwraps if wrapping set
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <returns>float</returns>
        public float GetAt(int i, int j) {
            UnwrapIndices(ref i, ref j);

            return Array[i + j * nx];
        }

        /// <summary>
        /// Checked access - unwraps if wrapping set
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <param name="val"></param>
        public void SetAt(int i, int j, float val) {
            UnwrapIndices(ref i, ref j);

            Array[i + j * nx] = val;
        }

        /// <summary>
        /// UnwrapIndices
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        private void UnwrapIndices(ref int i, ref int j) {
            if (wrap == false) return;

            while (i < 0) i += nx;

            while (j < 0) j += nz;

            i = i % nx;
            j = j % nz;
        }

        /// <summary>
        /// ! Unchecked access - no wrapping
        /// </summary>
        /// <param name="i"></param>
        /// <param name="j"></param>
        /// <returns>float</returns>
        public float this[int i, int j] {
            get { return Array[i + j * nx]; }
        }

        /// <summary>
        /// enables/disables wrapping
        /// </summary>
        public bool Wrap {
            get { return wrap; }
            set { wrap = value; }
        }

        /// <summary>
        /// Gets the 'x' size of the array
        /// </summary>
        public int Nx {
            get { return nx; }
        }

        /// <summary>
        /// Gets the 'y' size of the array
        /// </summary>
        public int Nz {
            get { return nz; }
        }

        /// <summary>
        /// Gets or Sets min
        /// </summary>
        public float Min {
            get {
                float min = Array[0];

                for (int i = 0; i < Array.Length; ++i)
                    if (Array[i] < min)
                        min = Array[i];
                return min;
            }
        }

        /// <summary>
        /// Gets or Sets max
        /// </summary>
        public float Max {
            get {
                float max = Array[0];

                for (int i = 0; i < Array.Length; ++i)
                    if (max < Array[i])
                        max = Array[i];
                return max;
            }
        }
    }
}