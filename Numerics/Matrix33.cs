using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics
{
    public struct Matrix33 : IEquatable<Matrix33>
    {
        internal readonly (Matrix3 m_11, Matrix3 m_12, Matrix3 m_21, Matrix3 m_22) data;

        public Matrix33(Matrix3 m_11, Matrix3 m_12, Matrix3 m_21, Matrix3 m_22)
            : this((m_11, m_12, m_21, m_22)) { }
        public Matrix33((Matrix3 m_11, Matrix3 m_12, Matrix3 m_21, Matrix3 m_22) data) : this()
        {
            this.data = data;
        }
        public static implicit operator Matrix33(float value) => Scalar(value);
        public static readonly Matrix33 Zero = new Matrix33(Matrix3.Zero, Matrix3.Zero, Matrix3.Zero, Matrix3.Zero);
        public static readonly Matrix33 Identity = new Matrix33(Matrix3.Identity, Matrix3.Zero, Matrix3.Zero, Matrix3.Identity);

        public static Matrix33 Scalar(float value) => new Matrix33(Matrix3.Scalar(value), Matrix3.Zero, Matrix3.Zero, Matrix3.Scalar(value));
        public static Matrix33 Diagonal(Matrix3 a11, Matrix3 a22) => new Matrix33(a11, Matrix3.Zero, Matrix3.Zero, a22);
        public static Matrix33 Symmetric(Matrix3 a11, Matrix3 a12, Matrix3 a22)
            => new Matrix33(a11, a12, a12.Transpose(), a22);

        public (Matrix3 a11, Matrix3 a12, Matrix3 a21, Matrix3 a22) Data => data;

        #region Properties
        public int RowCount => 2;
        public int ColumnCount => 2;

        /// <summary>
        /// Check of the number of rows equals the number of columns. Always returns <c>true</c>.
        /// </summary>
        public bool IsSquare => true;
        /// <summary>
        /// Check if the matrix is symmetric
        /// </summary>
        public bool IsSymmetric 
            => data.m_11.IsSymmetric && data.m_22.IsSymmetric 
            && data.m_12 == data.m_21.Transpose();
        /// <summary>
        /// Check if the matrix is skew-symmetric
        /// </summary>
        public bool IsSkewSymmetric
            => data.m_11.IsSkewSymmetric && data.m_22.IsSkewSymmetric
            && data.m_12 == -data.m_21.Transpose();
        /// <summary>
        /// Check if the matrix is diagonal
        /// </summary>
        public bool IsDiagonal
            => data.m_11.IsDiagonal && data.m_22.IsDiagonal
            && data.m_12.IsZero && data.m_21.IsZero;

        public float Determinant
        {
            get
            {
                // Determinant of block matrices, sources:
                //
                // 1. https://djalil.chafai.net/blog/2012/10/14/determinant-of-block-matrices/
                // 2. https://en.wikipedia.org/wiki/Determinant#Block_matrices
                //
                if (data.m_22.Determinant == 0) return 0;
                return data.m_22.Determinant
                    * (data.m_11 - data.m_12 * data.m_22.Solve(data.m_21)).Determinant;
            }
        }

        public float Trace => data.m_11.Trace + data.m_22.Trace;

        public Matrix3 this[int row, int column]
        {
            get
            {
                switch (2*row + column)
                {
                    case 0: return data.m_11;
                    case 1: return data.m_12;
                    case 2: return data.m_21;
                    case 3: return data.m_22;
                    default:
                        throw new ArgumentOutOfRangeException(nameof(column));
                }
            }
        }

        #endregion

        #region Algebra

        public static Matrix33 Add(Matrix33 a, Matrix33 b)
            => new Matrix33(
                a.data.m_11 + b.data.m_11,
                a.data.m_12 + b.data.m_12,
                a.data.m_21 + b.data.m_21,
                a.data.m_22 + b.data.m_22);

        public static Matrix33 Subtract(Matrix33 a, Matrix33 b)
            => new Matrix33(
                a.data.m_11 - b.data.m_11,
                a.data.m_12 - b.data.m_12,
                a.data.m_21 - b.data.m_21,
                a.data.m_22 - b.data.m_22);

        public static Matrix33 Negate(Matrix33 a)
            => new Matrix33(
                -a.data.m_11,
                -a.data.m_12,
                -a.data.m_21,
                -a.data.m_22);

        public static Matrix33 Scale(float f, Matrix33 a)
            => new Matrix33(
                f * a.data.m_11,
                f * a.data.m_12,
                f * a.data.m_21,
                f * a.data.m_22);

        public Matrix33 Transpose() => new Matrix33(
            data.m_11.Transpose(), data.m_21.Transpose(), 
            data.m_12.Transpose(), data.m_22.Transpose());

        public static Vector33 Product(Matrix33 a, Vector33 v)
            => new Vector33(
                a.data.m_11 * v.data.m_1 + a.data.m_12 * v.data.m_2,
                a.data.m_21 * v.data.m_1 + a.data.m_22 * v.data.m_2);
        public static Vector33 Product(Vector33 v, Matrix33 a)
            => new Vector33(
                a.data.m_11 * v.data.m_1 + a.data.m_21 * v.data.m_2,
                a.data.m_12 * v.data.m_1 + a.data.m_22 * v.data.m_2);

        public static Matrix33 Product(Matrix33 a, Matrix33 b)
            => new Matrix33(
                a.data.m_11 * b.data.m_11 + a.data.m_12 * b.data.m_21,
                a.data.m_11 * b.data.m_12 + a.data.m_12 * b.data.m_22,
                a.data.m_21 * b.data.m_11 + a.data.m_22 * b.data.m_21,
                a.data.m_21 * b.data.m_12 + a.data.m_22 * b.data.m_22);

        public static Matrix33 operator -(Matrix33 a) => Negate(a);
        public static Matrix33 operator *(float f, Matrix33 a) => Scale(f, a);
        public static Matrix33 operator *(Matrix33 a, float f) => Scale(f, a);
        public static Matrix33 operator /(Matrix33 a, float d) => Scale(1 / d, a);
        public static Matrix33 operator +(Matrix33 a, Matrix33 b) => Add(a, b);
        public static Matrix33 operator -(Matrix33 a, Matrix33 b) => Subtract(a, b);
        public static Matrix33 operator ~(Matrix33 a) => a.Transpose();

        public static Vector33 operator *(Matrix33 a, Vector33 v) => Product(a, v);
        public static Vector33 operator *(Vector33 v, Matrix33 a) => Product(v, a);
        public static Matrix33 operator *(Matrix33 a, Matrix33 b) => Product(a, b);

        public Vector33 Solve(Vector33 vector)
        {            
            var A_inv = data.m_11.Inverse();
            var D_inv = data.m_22.Inverse();
            var As = data.m_11 - data.m_12 * D_inv * data.m_21;
            var Ds = data.m_22 - data.m_21 * A_inv * data.m_12;
            var u = vector.data.m_1 - data.m_12 * D_inv * vector.data.m_2;
            var v = vector.data.m_2 - data.m_21 * A_inv * vector.data.m_1;

            return new Vector33(
                As.Solve(u),
                Ds.Solve(v));
        }
        public Matrix33 Solve(Matrix33 matrix)
        {
            var A_inv = data.m_11.Inverse();
            var D_inv = data.m_22.Inverse();
            var As = data.m_11 - data.m_12 * D_inv * data.m_21;
            var Ds = data.m_22 - data.m_21 * A_inv * data.m_12;

            var L = matrix.data.m_11 - data.m_12 * D_inv * matrix.data.m_21;
            var N = matrix.data.m_12 - data.m_12 * D_inv * matrix.data.m_22;
            var K = matrix.data.m_21 - data.m_21 * A_inv * matrix.data.m_11;
            var M = matrix.data.m_22 - data.m_21 * A_inv * matrix.data.m_12;

            return new Matrix33(
                As.Solve(L), As.Solve(N),
                Ds.Solve(K), Ds.Solve(M));
        }

        public Matrix33 Inverse()
            => Solve(Identity);

        #endregion

        #region Formatting
        public override string ToString() => $"Matrix33(" +
            $"{data.m_11.data.m_11},{data.m_11.data.m_12},{data.m_11.data.m_13}:{data.m_12.data.m_11},{data.m_12.data.m_12},{data.m_12.data.m_13}|" +
            $"{data.m_11.data.m_21},{data.m_11.data.m_22},{data.m_11.data.m_23}:{data.m_12.data.m_21},{data.m_12.data.m_22},{data.m_12.data.m_23}|" +
            $"{data.m_11.data.m_31},{data.m_11.data.m_32},{data.m_11.data.m_33}:{data.m_12.data.m_31},{data.m_12.data.m_32},{data.m_12.data.m_33}|" +
            $"{data.m_21.data.m_11},{data.m_21.data.m_12},{data.m_21.data.m_13}:{data.m_22.data.m_11},{data.m_22.data.m_12},{data.m_22.data.m_13}|" +
            $"{data.m_21.data.m_21},{data.m_21.data.m_22},{data.m_21.data.m_23}:{data.m_22.data.m_21},{data.m_22.data.m_22},{data.m_22.data.m_23}|" +
            $"{data.m_21.data.m_31},{data.m_21.data.m_32},{data.m_21.data.m_33}:{data.m_22.data.m_31},{data.m_22.data.m_32},{data.m_22.data.m_33})";
            
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Matrix33)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Matrix33 other)
            {
                return Equals(other);
            }
            return false;
        }

        public static bool operator ==(Matrix33 target, Matrix33 other) { return target.Equals(other); }
        public static bool operator !=(Matrix33 target, Matrix33 other) { return !(target == other); }


        /// <summary>
        /// Checks for equality among <see cref="Matrix33"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Matrix33"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Matrix33 other)
        {
            return data.Equals(other.data);
        }

        /// <summary>
        /// Calculates the hash code for the <see cref="Matrix33"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc = (-1521134295) * hc + data.GetHashCode();
                return hc;
            }
        }

        #endregion

        #region Collection
        /// <summary>
        /// Get the number of elements in the matrix.
        /// </summary>
        public int Count => 4;

        /// <summary>
        /// Get the elemetns as an array
        /// </summary>
        public Matrix3[] ToArray() => AsSpan().ToArray();

        /// <summary>
        /// Get the elements as a span
        /// </summary>
        public unsafe ReadOnlySpan<Matrix3> AsSpan()
        {
            fixed (Matrix3* ptr = &data.m_11)
            {
                return new ReadOnlySpan<Matrix3>(ptr, Count);
            }
        }
        #endregion


    }
}
