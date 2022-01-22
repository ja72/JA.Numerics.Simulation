using System;
using System.Numerics;

namespace JA.Numerics
{
    public struct Matrix2 : IEquatable<Matrix2>
    {
        internal readonly (float m_11, float m_12, float m_21, float m_22) data;

        public Matrix2(float m_11, float m_12, float m_21, float m_22) : this((m_11, m_12, m_21, m_22)) { }
        public Matrix2((float m_11, float m_12, float m_21, float m_22) data) : this()
        {
            this.data = data;
        }
        public static readonly Matrix2 Zero = new Matrix2(0, 0, 0, 0);
        public static readonly Matrix2 Identity = new Matrix2(1, 0, 0, 1);
        public static implicit operator Matrix2(float value) => Diagonal(value);

        public static Matrix2 Random(float minValue = 0, float maxValue = 1) => new Matrix2(
            Constants.Random(minValue, maxValue), Constants.Random(minValue, maxValue),
            Constants.Random(minValue, maxValue), Constants.Random(minValue, maxValue));
        public static Matrix2 Diagonal(Vector2 v) => Diagonal(v.X, v.Y);
        public static Matrix2 Diagonal(float d) => Diagonal(d, d);
        public static Matrix2 Diagonal(float d1, float d2)
            => new Matrix2(d1, 0, 0, d2);
        public static Matrix2 FromRows(Vector2 row1, Vector2 row2)
            => new Matrix2(row1.X, row1.Y, row2.X, row2.Y);
        public static Matrix2 FromColumns(Vector2 column1, Vector2 column2)
            => new Matrix2(column1.X, column2.X, column1.Y, column2.Y);

        #region Properties
        public (float a11, float a12, float a21, float a22) Data => data;

        /// <summary>
        /// Get the number of rows
        /// </summary>
        public int RowCount => 2;
        /// <summary>
        /// Get the number of columns
        /// </summary>
        public int ColumnCount => 2;
        /// <summary>
        /// Check of the number of rows equals the number of columns. Always returns <c>true</c>.
        /// </summary>
        public bool IsSquare => true;
        /// <summary>
        /// Check if all elements are zero
        /// </summary>
        public bool IsZero
            => data.m_11 == 0 && data.m_12 == 0
            && data.m_21 == 0 && data.m_22 == 0;
        /// <summary>
        /// Check if the matrix is symmetric
        /// </summary>
        public bool IsSymmetric => data.m_12 == data.m_21;
        /// <summary>
        /// Check if the matrix is skew-symmetric
        /// </summary>
        public bool IsSkewSymmetric
            => data.m_12 == -data.m_21
            && data.m_11 == 0 && data.m_22 == 0;
        /// <summary>
        /// Check if the matrix is diagonal
        /// </summary>
        public bool IsDiagonal
            => data.m_12 == 0 && data.m_21 == 0;

        public float Determinant { get => data.m_11 * data.m_22 - data.m_12 * data.m_21; }
        public float Trace { get => data.m_11 + data.m_22; }

        public Vector2 Row1 => new Vector2(data.m_11, data.m_12);
        public Vector2 Row2 => new Vector2(data.m_21, data.m_22);
        public Vector2 Column1 => new Vector2(data.m_11, data.m_21);
        public Vector2 Column2 => new Vector2(data.m_12, data.m_22);

        public float this[int row, int column]
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

        public static Matrix2 Add(Matrix2 a, Matrix2 b) 
            => new Matrix2(
                a.data.m_11 + b.data.m_11,
                a.data.m_12 + b.data.m_12,
                a.data.m_21 + b.data.m_21,
                a.data.m_22 + b.data.m_22);

        public static Matrix2 Subtract(Matrix2 a, Matrix2 b)
            => new Matrix2(
                a.data.m_11 - b.data.m_11,
                a.data.m_12 - b.data.m_12,
                a.data.m_21 - b.data.m_21,
                a.data.m_22 - b.data.m_22);

        public static Matrix2 Negate(Matrix2 a)
            => new Matrix2(
                -a.data.m_11,
                -a.data.m_12,
                -a.data.m_21,
                -a.data.m_22);

        public static Matrix2 Scale(float f, Matrix2 a)
            => new Matrix2(
                f * a.data.m_11,
                f * a.data.m_12,
                f * a.data.m_21,
                f * a.data.m_22);

        public Matrix2 Transpose() => new Matrix2(data.m_11, data.m_21, data.m_12, data.m_22);

        public static Vector2 Product(Matrix2 a, Vector2 v)
            => new Vector2(
                a.data.m_11 * v.X + a.data.m_12 * v.Y,
                a.data.m_21 * v.X + a.data.m_22 * v.Y);
        public static Vector2 Product(Vector2 v, Matrix2 a)
            => new Vector2(
                a.data.m_11 * v.X + a.data.m_21 * v.Y,
                a.data.m_12 * v.X + a.data.m_22 * v.Y);

        public static Matrix2 Product(Matrix2 a, Matrix2 b) 
            => new Matrix2(
                a.data.m_11 * b.data.m_11 + a.data.m_12 * b.data.m_21,
                a.data.m_11 * b.data.m_12 + a.data.m_12 * b.data.m_22,
                a.data.m_21 * b.data.m_11 + a.data.m_22 * b.data.m_21,
                a.data.m_21 * b.data.m_12 + a.data.m_22 * b.data.m_22);

        public static Matrix2 operator -(Matrix2 a) => Negate(a);
        public static Matrix2 operator *(float f, Matrix2 a) => Scale(f, a);
        public static Matrix2 operator *(Matrix2 a, float f) => Scale(f, a);
        public static Matrix2 operator /(Matrix2 a, float d) => Scale(1/d, a);
        public static Matrix2 operator +(Matrix2 a, Matrix2 b) => Add(a, b);
        public static Matrix2 operator -(Matrix2 a, Matrix2 b) => Subtract(a, b);
        public static Matrix2 operator ~(Matrix2 a) => a.Transpose();

        public static Vector2 operator *(Matrix2 a, Vector2 v) => Product(a, v);
        public static Vector2 operator *(Vector2 v, Matrix2 a) => Product(v, a);
        public static Matrix2 operator *(Matrix2 a, Matrix2 b) => Product(a, b);

        public Vector2 Solve(Vector2 v)
        {
            var d = Determinant;
            return new Vector2(
                (data.m_22 * v.X - data.m_12 * v.Y) / d,
                (data.m_11 * v.Y - data.m_21 * v.X) / d);
        }
        public Matrix2 Solve(Matrix2 m) 
            => FromColumns(
                Solve(m.Column1),
                Solve(m.Column2));

        public Matrix2 Inverse()
            => Solve(Identity);

        #endregion

        #region Formatting
        public override string ToString()
            => $"Matrix2{data}";
        #endregion

        #region IEquatable Members

        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(SymMatrix2)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Matrix2 item)
            {
                return Equals(item);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among <see cref="Matrix2"/> classes
        /// </summary>
        /// <returns>True if equal</returns>
        public bool Equals(Matrix2 other)
        {
            return data.Equals(other.data);
        }
        /// <summary>
        /// Calculates the hash code for the <see cref="Matrix2"/>
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
        public static bool operator ==(Matrix2 target, Matrix2 other) { return target.Equals(other); }
        public static bool operator !=(Matrix2 target, Matrix2 other) { return !target.Equals(other); }

        #endregion

        #region Collection
        /// <summary>
        /// Get the number of elements in the matrix.
        /// </summary>
        public int Count => 4;

        /// <summary>
        /// Get the elemetns as an array
        /// </summary>
        public float[] ToArray() => AsSpan().ToArray();

        /// <summary>
        /// Get the elements as a span
        /// </summary>
        public unsafe ReadOnlySpan<float> AsSpan()
        {
            fixed (float* ptr = &data.m_11)
            {
                return new ReadOnlySpan<float>(ptr, Count);
            }
        }

        #endregion

    }
}
