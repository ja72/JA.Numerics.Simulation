using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics
{
    public struct Matrix3 : IEquatable<Matrix3>
    {
        internal readonly (float m_11, float m_12, float m_13, float m_21, float m_22, float m_23, float m_31, float m_32, float m_33) data;

        #region Factory
        public Matrix3(float m_11, float m_12, float m_13, float m_21, float m_22, float m_23, float m_31, float m_32, float m_33)
            : this((m_11, m_12, m_13, m_21, m_22, m_23, m_31, m_32, m_33)) { }
        public Matrix3((float m_11, float m_12, float m_13, float m_21, float m_22, float m_23, float m_31, float m_32, float m_33) data) : this()
        {
            this.data = data;
        }
        public static implicit operator Matrix3(float value) => Scalar(value);
        public static readonly Matrix3 Zero = Scalar(0);
        public static readonly Matrix3 Identity = Scalar(1);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Diagonal(float a11, float a22, float a33) 
            => new Matrix3(a11, 0, 0, 0, a22, 0, 0, 0, a33);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Diagonal(Vector3 vector)
            => Diagonal(vector.X, vector.Y, vector.Z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Scalar(float value)
            => Diagonal(value,value,value);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Symmetric(float a11, float a12, float a13, float a22, float a23, float a33)
            => new Matrix3(
                a11, a12, a13, 
                a12, a22, a23, 
                a13, a23, a33);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 SkewSymmetric(float a12, float a13, float a23)
            => new Matrix3(
                0, a12, a13,
                -a12, 0, a23,
                -a13, -a23, 0);

        public static Matrix3 Random(float minValue = 0, float maxValue = 1)
            => FromRows(
                LinearAlgebra.RandomVector(minValue, maxValue),
                LinearAlgebra.RandomVector(minValue, maxValue),
                LinearAlgebra.RandomVector(minValue, maxValue));

        public static Matrix3 CreateRotationX(float theta)
        {
            float c = (float)Math.Cos(theta), s = (float)Math.Sin(theta);
            return new Matrix3((1, 0, 0, 0, c, -s, 0, s, c));
        }
        public static Matrix3 CreateRotationY(float theta)
        {
            float c = (float)Math.Cos(theta), s = (float)Math.Sin(theta);
            return new Matrix3((c, 0, s, 0, 1, 0, -s, 0, c));
        }
        public static Matrix3 CreateRotationZ(float theta)
        {
            float c = (float)Math.Cos(theta), s = (float)Math.Sin(theta);
            return new Matrix3((c, -s, 0, s, c, 0, 0, 0, 1));
        }

        public static Matrix3 CreateRotation(Vector3 axis, float theta)
        {
            float m = axis.Length();
            (float x, float y, float z) = (axis.X / m, axis.Y / m, axis.Z / m);
            (float c, float s) = ((float)Math.Cos(theta), (float)Math.Sin(theta));
            float v = 1 - c;
            (float xx, float yy, float zz) = (x * x, y * y, z * z);
            return new Matrix3(
                (c * (yy + zz) + xx, v * x * y - s * z, s * y + v * x * z,
                s * z + v * x * y, c * (xx + zz) + yy, v * y * z - s * x,
                v * x * z - s * y, s * x + v * y * z, c * (xx + yy) + zz));
        }
        public static Matrix3 CreateRotation(Quaternion q)
        {
            float m2 = q.LengthSquared(), im2 = 1/m2;
            float x = q.X, y = q.Y, z = q.Z, w = q.W;
            return new Matrix3(
                1 - 2 * (y * y + z * z) * im2, 2 * (x * y - w * z) * im2, 2 * (w * y + x * z) * im2,
                2 * (w * z + x * y) * im2, 1 - 2 * (x * x + z * z) * im2, 2 * (y * z - w * x) * im2,
                2 * (x * z - w * y) * im2, 2 * (w * x + y * z) * im2, 1 - 2 * (x * x + y * y) * im2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 FromColumns(Vector3 column1, Vector3 column2, Vector3 column3) 
            => new Matrix3(
                (column1.X, column2.X, column3.X,
                column1.Y, column2.Y, column3.Y,
                column1.Z, column2.Z, column3.Z));
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 FromRows(Vector3 row1, Vector3 row2, Vector3 row3)
            => new Matrix3(
                (row1.X, row1.Y, row1.Z,
                row2.X, row2.Y, row2.Z,
                row3.X, row3.Y, row3.Z));


        public Quaternion ToRotation()
        {
            float s = 0.5f * (float)Math.Sqrt((
                (data.m_32 - data.m_23) * (data.m_32 - data.m_23)
                + (data.m_13 - data.m_31) * (data.m_13 - data.m_31)
                + (data.m_21 - data.m_12) * (data.m_21 - data.m_12))
                / (3 - data.m_11 - data.m_22 - data.m_33));

            Vector3 v = 1 / (4 * s) * (new Vector3(
                data.m_32 - data.m_23,
                data.m_13 - data.m_31,
                data.m_21 - data.m_12));

            float m2 = s * s + v.LengthSquared();
            float m = (float)Math.Sqrt(m2);
            s /= m;
            v /= m;

            return new Quaternion(v, s);
        }

        #endregion

        #region Properties

        public (float a11, float a12, float a13, float a21, float a22, float a23, float a31, float a32, float a33) Data => data;
        /// <summary>
        /// Get the number of rows
        /// </summary>
        public int RowCount => 3;
        /// <summary>
        /// Get the number of columns
        /// </summary>
        public int ColumnCount => 3;

        /// <summary>
        /// Check of the number of rows equals the number of columns. Always returns <c>true</c>.
        /// </summary>
        public bool IsSquare => true;
        /// <summary>
        /// Check if all elements are zero
        /// </summary>
        public bool IsZero
            => data.m_11 == 0 && data.m_12 == 0 && data.m_13 == 0
            && data.m_21 == 0 && data.m_22 == 0 && data.m_23 == 0
            && data.m_31 == 0 && data.m_32 == 0 && data.m_33 == 0;
        /// <summary>
        /// Check if the matrix is symmetric
        /// </summary>
        public bool IsSymmetric => data.m_12 == data.m_21 && data.m_13 == data.m_31 && data.m_23 == data.m_32;
        /// <summary>
        /// Check if the matrix is skew-symmetric
        /// </summary>
        public bool IsSkewSymmetric
            => data.m_12 == -data.m_21 && data.m_13 == -data.m_31 && data.m_23 == -data.m_32
            && data.m_11 == 0 && data.m_22 == 0 && data.m_33 == 0;
        /// <summary>
        /// Check if the matrix is diagonal
        /// </summary>
        public bool IsDiagonal
            => data.m_12 == 0 && data.m_13 == 0 && data.m_23 == 0
            && data.m_21 == 0 && data.m_31 == 0 && data.m_32 == 0;

        public Vector3 Row1 { get => new Vector3(data.m_11, data.m_12, data.m_13); }
        public Vector3 Row2 { get => new Vector3(data.m_21, data.m_22, data.m_23); }
        public Vector3 Row3 { get => new Vector3(data.m_31, data.m_32, data.m_33); }
        public Vector3 Column1 { get => new Vector3(data.m_11, data.m_21, data.m_31); }
        public Vector3 Column2 { get => new Vector3(data.m_12, data.m_22, data.m_32); }
        public Vector3 Column3 { get => new Vector3(data.m_13, data.m_23, data.m_33); }

        public float M11 { get => data.m_11; }
        public float M12 { get => data.m_12; }
        public float M13 { get => data.m_13; }
        public float M21 { get => data.m_21; }
        public float M22 { get => data.m_22; }
        public float M23 { get => data.m_23; }
        public float M31 { get => data.m_31; }
        public float M32 { get => data.m_32; }
        public float M33 { get => data.m_33; }

        /// <summary>
        /// Matrix Determinant
        /// </summary>
        public float Determinant
            => data.m_11 * (data.m_22 * data.m_33 - data.m_23 * data.m_32)
            + data.m_12 * (data.m_23 * data.m_31 - data.m_21 * data.m_33)
            + data.m_13 * (data.m_21 * data.m_32 - data.m_22 * data.m_31);

        /// <summary>
        /// Gets the trace (sum of diagonals) of the matrix.
        /// </summary>
        public float Trace => data.m_11 + data.m_22 + data.m_33;

        public float this[int row, int column]
        {
            get
            {
                switch (3*row + column)
                {
                    case 0: return data.m_11;
                    case 1: return data.m_12;
                    case 2: return data.m_13;
                    case 3: return data.m_21;
                    case 4: return data.m_22;
                    case 5: return data.m_23;
                    case 6: return data.m_31;
                    case 7: return data.m_32;
                    case 8: return data.m_33;
                    default:
                        throw new ArgumentOutOfRangeException(nameof(column));
                }
            }
        }

        #endregion

        #region Algebra
        public static Matrix3 Negate(Matrix3 a) 
            => new Matrix3(
            (-a.data.m_11, -a.data.m_12, -a.data.m_13,
            -a.data.m_21, -a.data.m_22, -a.data.m_23,
            -a.data.m_31, -a.data.m_32, -a.data.m_33));
        public static Matrix3 Add(Matrix3 a, Matrix3 b) 
            => new Matrix3(
                (a.data.m_11 + b.data.m_11, a.data.m_12 + b.data.m_12, a.data.m_13 + b.data.m_13,
                 a.data.m_21 + b.data.m_21, a.data.m_22 + b.data.m_22, a.data.m_23 + b.data.m_23,
                 a.data.m_31 + b.data.m_31, a.data.m_32 + b.data.m_32, a.data.m_33 + b.data.m_33));
        public static Matrix3 Subtract(Matrix3 a, Matrix3 b)
            => new Matrix3(
                (a.data.m_11 - b.data.m_11, a.data.m_12 - b.data.m_12, a.data.m_13 - b.data.m_13,
                 a.data.m_21 - b.data.m_21, a.data.m_22 - b.data.m_22, a.data.m_23 - b.data.m_23,
                 a.data.m_31 - b.data.m_31, a.data.m_32 - b.data.m_32, a.data.m_33 - b.data.m_33));
        public static Matrix3 Scale(float factor, Matrix3 a) 
            => new Matrix3(
                (factor*a.data.m_11, factor*a.data.m_12, factor*a.data.m_13,
                factor*a.data.m_21, factor*a.data.m_22, factor*a.data.m_23,
                factor*a.data.m_31, factor*a.data.m_32, factor*a.data.m_33));
        public static Vector3 Product(Matrix3 a, Vector3 b) 
            => new Vector3(
                a.data.m_11 * b.X + a.data.m_12 * b.Y + a.data.m_13 * b.Z,
                a.data.m_21 * b.X + a.data.m_22 * b.Y + a.data.m_23 * b.Z,
                a.data.m_31 * b.X + a.data.m_32 * b.Y + a.data.m_33 * b.Z);

        public static Matrix3 Product(Matrix3 a, Matrix3 b)
            => new Matrix3(
                a.data.m_11 * b.data.m_11 + a.data.m_12 * b.data.m_21 + a.data.m_13 * b.data.m_31,
                a.data.m_11 * b.data.m_12 + a.data.m_12 * b.data.m_22 + a.data.m_13 * b.data.m_32,
                a.data.m_11 * b.data.m_13 + a.data.m_12 * b.data.m_23 + a.data.m_13 * b.data.m_33,
                a.data.m_21 * b.data.m_11 + a.data.m_22 * b.data.m_21 + a.data.m_23 * b.data.m_31,
                a.data.m_21 * b.data.m_12 + a.data.m_22 * b.data.m_22 + a.data.m_23 * b.data.m_32,
                a.data.m_21 * b.data.m_13 + a.data.m_22 * b.data.m_23 + a.data.m_23 * b.data.m_33,
                a.data.m_31 * b.data.m_11 + a.data.m_32 * b.data.m_21 + a.data.m_33 * b.data.m_31,
                a.data.m_31 * b.data.m_12 + a.data.m_32 * b.data.m_22 + a.data.m_33 * b.data.m_32,
                a.data.m_31 * b.data.m_13 + a.data.m_32 * b.data.m_23 + a.data.m_33 * b.data.m_33);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix3 Transpose()
        {
            return new Matrix3(
                (data.m_11, data.m_21, data.m_31,
                data.m_12, data.m_22, data.m_32,
                data.m_13, data.m_23, data.m_33));
        }

        public Matrix3 Inverse(float factor=1)
        {
            float d = Determinant;
            float inv_d = factor / d;
            return new Matrix3(
                inv_d * (data.m_22 * data.m_33 - data.m_23 * data.m_32),
                inv_d * (data.m_13 * data.m_32 - data.m_12 * data.m_33),
                inv_d * (data.m_12 * data.m_23 - data.m_13 * data.m_22),
                inv_d * (data.m_23 * data.m_31 - data.m_21 * data.m_33),
                inv_d * (data.m_11 * data.m_33 - data.m_13 * data.m_31),
                inv_d * (data.m_13 * data.m_21 - data.m_11 * data.m_23),
                inv_d * (data.m_21 * data.m_32 - data.m_22 * data.m_31),
                inv_d * (data.m_12 * data.m_31 - data.m_11 * data.m_32),
                inv_d * (data.m_11 * data.m_22 - data.m_12 * data.m_21));
        }

        public Vector3 Solve(Vector3 other)
        {
            float D = Determinant;
            float ID = 1 / D;
            float x = other.X * ID, y = other.Y * ID, z = other.Z * ID;
            return new Vector3(
                  data.m_22 * data.m_33 * x - data.m_23 * data.m_32 * x
                + data.m_13 * data.m_32 * y - data.m_12 * data.m_33 * y
                + data.m_12 * data.m_23 * z - data.m_13 * data.m_22 * z,
                  data.m_23 * data.m_31 * x - data.m_21 * data.m_33 * x
                + data.m_11 * data.m_33 * y - data.m_13 * data.m_31 * y
                + data.m_13 * data.m_21 * z - data.m_11 * data.m_23 * z,
                  data.m_21 * data.m_32 * x - data.m_22 * data.m_31 * x
                + data.m_12 * data.m_31 * y - data.m_11 * data.m_32 * y
                + data.m_11 * data.m_22 * z - data.m_12 * data.m_21 * z);

        }
        public Matrix3 Solve(Matrix3 other) => Inverse() * other;

        public Matrix3 Symmetric() => (this + Transpose()) / 2;
        public Matrix3 SkewSymmetric() => (this - Transpose()) / 2;

        public float MaxAbs() => ToArray().Max((x) => Math.Abs(x));
        public float MinAbs() => ToArray().Where((x) => x != 0).Min((x) => Math.Abs(x));

        #endregion

        #region Eigenvalues
        /// <summary>
        /// Calculates the three eigenvalues analytically.
        /// </summary>
        /// <remarks>
        /// Code taken from:
        /// https://www.mpi-hd.mpg.de/personalhomes/globes/3x3/index.html
        /// </remarks>
        /// <returns>A vector containing the three eigenvalues.</returns>
        public Vector3 GetEigenValues()
        {
            //      Determine coefficients of characteristic polynomial. We write
            //      | A   D   F  |
            // A =  | D*  B   E  |
            //      | F*  E*  C  |

            var de = data.m_12 * data.m_23;
            var dd = data.m_12 * data.m_12;
            var ee = data.m_23 * data.m_23;
            var ff = data.m_13 * data.m_13;
            var m = data.m_11 + data.m_22 + data.m_33;
            var c1 = (data.m_11 * data.m_22 + data.m_11 * data.m_33 + data.m_22 * data.m_33) - (dd + ee + ff);
            var c0 = data.m_33 * dd + data.m_11 * ee + data.m_22 * ff - data.m_11 * data.m_22 * data.m_33 - 2.0 * data.m_13 * de;

            var p = m * m - 3.0 * c1;
            var q = m * (p - (3.0 / 2.0) * c1) - (27.0 / 2.0) * c0;
            var sqrt_p = (float) Math.Sqrt(Math.Abs(p));

            var sqrt_z = (float) Math.Sqrt(Math.Abs(27.0 * (0.25 * c1 * c1 * (p - c1) + c0 * (q + 27.0 / 4.0 * c0))));
            var phi = (1 / 3f) * (float) Math.Atan2(sqrt_z, q);

            var c = sqrt_p * (float) Math.Cos(phi);
            var s = sqrt_p * (float)( Math.Abs(Math.Sin(phi))/ Math.Sqrt(3));

            var w = (1 / 3f) * (m - c);

            // sort the eigenvalues
            if (c >= s)
            {
                return new Vector3(
                    w - s,
                    w + s,
                    w + c);
            }
            else if (c >= -s)
            {
                return new Vector3(
                    w - s,
                    w + c,
                    w + s);
            }
            else
            {
                return new Vector3(
                    w + c,
                    w - s,
                    w + s);
            }
        }

        public Matrix3 GetEigenVectors() => GetEigenVectors(GetEigenValues());
        public Matrix3 GetEigenVectors(Vector3 eigenValues)
        {
            Vector3 ev1 = GetEigenVector(eigenValues.X).Unit();
            Vector3 ev2 = GetEigenVector(eigenValues.Y).Unit();
            Vector3 ev3 = GetEigenVector(eigenValues.Z).Unit();

            return FromColumns(ev1, ev2, ev3);
        }
        Vector3 GetEigenVector(float w)
        {
            return new Vector3(
                  data.m_12 * (data.m_23 - data.m_33 + w) - data.m_13 * (data.m_22 - data.m_23 - w)
                + data.m_22 * (data.m_33 - w) - data.m_23 * data.m_23 - w * (data.m_33 - w),
                -data.m_11 * (data.m_23 - data.m_33 + w) + data.m_12 * (data.m_13 - data.m_33 + w)
                - data.m_13 * data.m_13 + data.m_13 * data.m_23 + w * (data.m_23 - data.m_33 + w),
                  data.m_11 * (data.m_22 - data.m_23 - w) - data.m_12 * data.m_12 + data.m_12 * (data.m_13 + data.m_23)
                + data.m_13 * (w - data.m_22) - w * (data.m_22 - data.m_23 - w));
        }

        #endregion

        #region Operators
        public static Matrix3 operator +(Matrix3 a, Matrix3 b) => Add(a, b);
        public static Matrix3 operator -(Matrix3 a, Matrix3 b) => Subtract(a, b);
        public static Matrix3 operator -(Matrix3 a) => Negate(a);
        public static Matrix3 operator *(float factor, Matrix3 a) => Scale(factor, a);
        public static Matrix3 operator *(Matrix3 a, float factor) => Scale(factor, a);
        public static Vector3 operator *(Matrix3 a, Vector3 v) => Product(a, v);
        public static Matrix3 operator *(Matrix3 a, Matrix3 b) => Product(a, b);
        public static Vector3 operator *(Vector3 v, Matrix3 a) => Product(a.Transpose(), v);
        public static Matrix3 operator /(Matrix3 a, float divisor) => Scale(1 / divisor, a);
        public static Matrix3 operator ~(Matrix3 a) => a.Transpose();
        public static Matrix3 operator !(Matrix3 a) => a.Inverse();
        #endregion

        #region Formatting
        public override string ToString() => $"Matrix3(" +
            $"{data.m_11},{data.m_12},{data.m_13}|" +
            $"{data.m_21},{data.m_22},{data.m_23}|" +
            $"{data.m_31},{data.m_32},{data.m_33})";
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Matrix3)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Matrix3 other)
            {
                return Equals(other);
            }
            return false;
        }


        /// <summary>
        /// Checks for equality among <see cref="Matrix3"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Matrix3"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Matrix3 other)
        {
            return data.Equals(other.data);
        }

        public static bool operator ==(Matrix3 target, Matrix3 other) { return target.Equals(other); }
        public static bool operator !=(Matrix3 target, Matrix3 other) { return !(target == other); }

        /// <summary>
        /// Calculates the hash code for the <see cref="Matrix3"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            return data.GetHashCode();
        }

        #endregion

        #region Collection
        /// <summary>
        /// Get the number of elements in the matrix.
        /// </summary>
        public int Count => 9;

        /// <summary>
        /// Get the elements as an array
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
