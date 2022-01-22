using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Security.Cryptography.X509Certificates;
using System.Security.Principal;
using System.Text;

namespace JA.Numerics
{

    /// <summary>
    /// Performs a LUP decomposition of a square matrix for use in linear system
    /// solving, determinant evaluation and other numeric procedures.
    /// </summary>
    /// <remarks>
    /// Code taken from http://msdn.microsoft.com/en-us/magazine/jj863137.aspx
    /// </remarks>
    public static class JaggedMatrixLU
    {

        public static bool MatrixDecompose(this float[][] matrix,
            out float[][] lu, out int[] perm, out int toggle)
        {
            Contract.Requires(matrix != null);
            Contract.Requires(matrix.Length > 0);
            Contract.Ensures(Contract.ValueAtReturn(out lu).Length == matrix.Length);
            Contract.Ensures(Contract.ForAll(lu, (row) => row.Length == matrix[0].Length));
            Contract.Ensures(Contract.ValueAtReturn(out perm).Length == matrix.Length);

            // Doolittle LUP decomposition.
            // assumes matrix is square.
            int n = matrix.Length; // convenience
            lu = Copy(matrix);
            perm = Enumerable.Range(0,n).ToArray();
            toggle = 1;
            for (int j = 0; j < n - 1; ++j) // each column
            {
                float colMax = Math.Abs(lu[j][j]); // largest val col j
                int pRow = j;
                for (int i = j + 1; i < n; ++i)
                {
                    var row_i = lu[i];
                    if (row_i[j] > colMax)
                    {
                        colMax = row_i[j];
                        pRow = i;
                    }
                }
                if (pRow != j) // swap rows
                {
                    Swap(ref lu[pRow], ref lu[j]);
                    Swap(ref perm[pRow], ref perm[j]);
                    toggle = -toggle; // row-swap toggle
                }
                var row_j = lu[j];
                var lu_jj = row_j[j];
                if (Math.Abs(lu[j][j]) <= LinearAlgebra.ZeroTolerance)
                {
                    return false; // consider a throw
                }
                for (int i = j + 1; i < n; ++i)
                {
                    var row_i = lu[i];
                    var lu_ij = row_i[j];
                    lu_ij /= lu_jj;
                    row_i[j] = lu_ij;
                    for (int k = j + 1; k < n; ++k)
                    {
                        row_i[k] -= lu_ij * row_j[k];
                    }
                    lu[i] = row_i;
                }
            } // maj column loop
            return true;
        }
        static bool HelperSolve(float[][] luMatrix, float[] b, out float[] x)
        {
            // solve luMatrix * x = b
            Contract.Requires(luMatrix != null);
            Contract.Requires(b != null);
            Contract.Requires(luMatrix.Length > 0);
            Contract.Requires(luMatrix.Length == b.Length);
            Contract.Ensures(Contract.ValueAtReturn<float[]>(out _).Length == luMatrix[0].Length);
            int n = luMatrix.Length;
            x = b.Clone() as float[];

            for (int i = 1; i < n; ++i)
            {
                float sum = x[i];
                var row = luMatrix[i];
                for (int j = 0; j < i; ++j)
                {
                    sum -= row[j] * x[j];
                }
                x[i] = sum;
            }
            var d = luMatrix[n - 1][n - 1];
            if (Math.Abs(d) == 0)
            {
                return false;
            }

            x[n - 1] /= d;
            for (int i = n - 2; i >= 0; --i)
            {
                var row = luMatrix[i];
                d = row[i];
                if (Math.Abs(d) == 0)
                {
                    return false;
                }

                float sum = x[i];
                for (int j = i + 1; j < n; ++j)
                {
                    sum -= row[j] * x[j];
                }
                x[i] = sum / d;
            }
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Swap<T>(ref T x, ref T y)
        {
            var t = x;
            x = y;
            y = t;
        }
        static bool HelperSolve(float[][] luMatrix, float[][] B, out float[][] X)
        {
            // solve luMatrix * X = B
            Contract.Requires(luMatrix != null);
            Contract.Requires(B != null);
            Contract.Requires(B.Length > 0);
            Contract.Requires(luMatrix.Length == B.Length);
            Contract.Ensures(Contract.ValueAtReturn<float[][]>(out _).Length == luMatrix[0].Length);
            int n = luMatrix.Length;
            int m = B[0].Length;
            X = new float[B.Length][];
            for (int i = 0; i < B.Length; i++)
            {
                X[i] = B[i].Clone() as float[];
            }
            for (int k = 0; k < m; k++)
            {
                for (int i = 1; i < n; ++i)
                {
                    float sum = X[i][k];
                    for (int j = 0; j < i; ++j)
                    {
                        sum -= luMatrix[i][j] * X[j][k];
                    }
                    X[i][k] = sum;
                }
                var d = luMatrix[n - 1][n - 1];
                if (Math.Abs(d) == 0)
                {
                    return false;
                }

                X[n - 1][k] /= d;
                for (int i = n - 2; i >= 0; --i)
                {
                    d = luMatrix[i][i];
                    if (Math.Abs(d) == 0)
                    {
                        return false;
                    }

                    float sum = X[i][k];
                    for (int j = i + 1; j < n; ++j)
                    {
                        sum -= luMatrix[i][j] * X[j][k];
                    }
                    X[i][k] = sum / d;
                }
            }
            return true;
        }
        public static float[][] MatrixInverse(this float[][] matrix)
        {
            Contract.Requires(matrix != null);
            Contract.Requires(matrix.Length > 0);
            Contract.Ensures(Contract.Result<float[][]>().Length == matrix.Length);
            Contract.Ensures(Contract.Result<float[][]>()[0].Length == matrix[0].Length);
            if (MatrixInverse(matrix, out float[][] inverse))
            {
                return inverse;
            }
            throw new ArgumentException("Singular Matrix", nameof(matrix));
        }
        public static bool MatrixInverse(this float[][] matrix, out float[][] inverse)
        {
            Contract.Requires(matrix != null);
            Contract.Requires(matrix.Length > 0);
            Contract.Requires(matrix[0].Length == matrix.Length);
            if (!MatrixDecompose(matrix, out float[][] lu, out int[] perm, out _))
            {
                inverse = Copy(matrix);
                return false;
            }
            return LuInverse(lu, perm, out inverse);
        }
        internal static bool LuInverse(this float[][] lu, int[] perm, out float[][] inverse)
        {
            int n = lu.Length;
            inverse = new float[n][];
            for (int j = 0; j < n; j++)
            {
                inverse[j] = new float[n];
            }
            float[] b = new float[n];
            for (int i = 0; i < n; ++i)
            {
                for (int j = 0; j < n; ++j)
                {
                    if (i == perm[j])
                    {
                        b[j] = 1;
                    }
                    else
                    {
                        b[j] = 0;
                    }
                }
                if (HelperSolve(lu, b, out float[] x))
                {
                    for (int j = 0; j < n; ++j)
                    {
                        inverse[j][i] = x[j];
                    }
                }
                else
                {
                    return false;
                }
            }
            return true;
        }
        public static float MatrixDeterminant(this float[][] matrix)
        {
            Contract.Requires(matrix != null);
            Contract.Requires(matrix.Length > 0);
            Contract.Requires(matrix[0].Length == matrix.Length);
            if (!MatrixDecompose(matrix, out float[][] lum, out _, out int toggle))
            {
                throw new ArgumentException("Unable to compute MatrixDeterminant", nameof(matrix));
            }
            return LuDeterminant(lum, toggle);
        }
        internal static float LuDeterminant(float[][] lu, int toggle)
        {
            float result = toggle;
            for (int i = 0; i < lu.Length; ++i)
            {
                result *= lu[i][i];
            }
            return result;
        }
        public static float[] SystemSolve(this float[][] matrix, float[] b)
        {
            Contract.Requires(matrix != null);
            Contract.Requires(b != null);
            Contract.Requires(matrix.Length > 0);
            Contract.Requires(matrix.Length == b.Length);
            Contract.Ensures(Contract.Result<float[]>().Length == matrix[0].Length);
            if (SystemSolve(matrix, b, out float[] x))
            {
                return x;
            }
            throw new ArgumentException("Singular Matrix", nameof(matrix));
        }
        public static bool SystemSolve(this float[][] matrix, float[] b, out float[] x)
        {
            Contract.Requires(matrix != null);
            Contract.Requires(b != null);
            Contract.Requires(matrix.Length > 0);
            Contract.Requires(matrix[0].Length == matrix.Length);
            Contract.Requires(b.Length == matrix.Length);
            Contract.Ensures(Contract.ValueAtReturn<float[]>(out _).Length == matrix[0].Length);
            // Solve Ax = b
            //int n = matrix.Length;
            if (!MatrixDecompose(matrix, out float[][] lum, out int[] perm, out _))
            {
                x = Array.Empty<float>();
                return false;
            }
            return LuSystemSolve(lum, perm, b, out x);
        }
        internal static bool LuSystemSolve(this float[][] lu, int[] perm, float[] b, out float[] x)
        {
            float[] bp = new float[b.Length];
            for (int i = 0; i < lu.Length; ++i)
            {
                bp[i] = b[perm[i]];
            }
            if (!HelperSolve(lu, bp, out x))
            {
                x = Array.Empty<float>();
                return false;
            }
            return true;
        }
        public static float[][] SystemSolve(this float[][] matrix, float[][] B)
        {
            if (SystemSolve(matrix, B, out float[][] x))
            {
                return x;
            }
            throw new ArgumentException("Singular Matrix", nameof(matrix));
        }
        public static bool SystemSolve(this float[][] matrix, float[][] B, out float[][] X)
        {
            Contract.Requires(matrix != null);
            Contract.Requires(matrix.Length > 0);
            Contract.Requires(matrix[0].Length == matrix.Length);
            Contract.Requires(B != null);
            Contract.Requires(B.Length == matrix.Length);
            if (!MatrixDecompose(matrix, out float[][] lum, out int[] perm, out _))
            {
                X = Array.Empty<float[]>();
                return false;
            }
            return LuSystemSolve(lum, perm, B, out X);
        }
        internal static bool LuSystemSolve(this float[][] lu, int[] perm, float[][] B, out float[][] X)
        {
            float[][] Bp = new float[B.Length][];
            for (int i = 0; i < lu.Length; ++i)
            {
                Bp[i] = B[perm[i]];
            }
            if (!HelperSolve(lu, Bp, out X))
            {
                X = Array.Empty<float[]>();
                return false;
            }
            return true;
        }
        public static float[][] PermArrayToMatrix(int[] perm)
        {
            Contract.Requires(perm != null);
            Contract.Assume(perm.Length > 0);
            // Doolittle perm array to corresponding matrix
            int n = perm.Length;
            float[][] result = new float[n][];
            for (int i = 0; i < n; ++i)
            {
                result[i] = new float[n];
                result[i][perm[i]] = 1;
            }
            return result;
        }
        public static float[][] UnPermute(float[][] luProduct, int[] perm)
        {
            Contract.Requires(luProduct != null);
            Contract.Requires(perm != null);
            Contract.Requires(luProduct.Length == perm.Length);

            float[][] result = Copy(luProduct);
            int[] unperm = new int[perm.Length];
            for (int i = 0; i < perm.Length; ++i)
            {
                unperm[perm[i]] = i; // create un-perm array
            }
            for (int r = 0; r < luProduct.Length; ++r) // each row
            {
                result[r] = luProduct[unperm[r]];
            }
            return result;
        }

        public static float[][] Copy(float[][] matrix)
        {
            var result = new float[matrix.Length][];
            for (int i = 0; i < result.Length; i++)
            {
                var row = new float[matrix[i].Length];
                Array.Copy(matrix[i], row, row.Length);
                result[i] = row;
            }
            return result;
        }

    }
}
