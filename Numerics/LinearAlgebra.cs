using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics
{
    public static class LinearAlgebra
    {
        public const float ZeroTolerance = 1e-6f;

        public static Random RandomNumberGenerator { get; } = new Random();

        #region Arrays
        public static float[] RandomArray(int size, float minValue = 0, float maxValue = 1)
        {
            var data = new float[size];
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = minValue + (float)RandomNumberGenerator.NextDouble() * (maxValue - minValue);
            }
            return data;
        }
        public static double[] RandomArray(int size, double minValue = 0, double maxValue = 1)
        {
            var data = new double[size];
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = minValue + RandomNumberGenerator.NextDouble() * (maxValue - minValue);
            }
            return data;
        }

        public static bool Equals(float[] a, float[] b)
        {
            if (a.Length != b.Length) return false;
            for (int i = 0; i < a.Length; i++)
            {
                if (a[i] != b[i]) return false;
            }
            return true;
        }
        public static bool Near(float[] a, float[] b, float tol = ZeroTolerance)
        {
            if (a.Length != b.Length) return false;
            for (int i = 0; i < a.Length; i++)
            {
                if (Math.Abs(b[i] - a[i]) > ZeroTolerance) return false;
            }
            return true;
        }

        public static int GetHashCode<TData>(params TData[] arguments)
        {
            unchecked
            {
                int hc = -1817952719;
                for (int i = 0; i < arguments.Length; i++)
                {
                    hc = (-1521134295) * hc + arguments[i].GetHashCode();
                }
                return hc;
            }
        }

        #endregion

        #region Comparisons

        public static AbsFloatComparer AbsFloat(float delta)
            => new AbsFloatComparer(delta);

        #endregion

        #region Planar

        public static Matrix2 Mmoi(this Vector2 position)
            => Dot(position, position) - Outer(position, position);

        public static Vector2 Rotate(this Vector2 vector, float angle)
            => angle != 0
                ? Vector2.Transform(vector, Matrix3x2.CreateRotation(angle))
                : vector;
        public static Vector2 Rotate(this Vector2 vector, Quaternion rotation)
            => Vector2.Transform(vector, rotation);
        public static float Dot(this Vector2 vector, Vector2 other)
            => Vector2.Dot(vector, other);
        public static Matrix2 Outer(this Vector2 vector, Vector2 other)
            => new Matrix2(
                vector.X * other.X, vector.X * other.Y,
                vector.Y * other.X, vector.Y * other.Y);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float[] ToArray(this Vector2 vector) => new[] { vector.X, vector.Y };
        /// <summary>
        /// Vector/Vector cross product.
        /// <code>[rx,ry] × [fx,fy] = rx*fy-ry*fx</code>
        /// </summary>
        /// <param name="vector">The planar vector.</param>
        /// <param name="other">The other planar vector.</param>
        /// <returns>A scalar</returns>
        public static float Cross(this Vector2 vector, Vector2 other)
            => vector.X * other.Y - vector.Y * other.X;

        /// <summary>
        /// Vector/scalar cross product.
        /// <code>[x,y] × ω = [y*ω, -x*ω]</code>
        /// </summary>
        /// <param name="planar">The planar vector.</param>
        /// <param name="normal">The scalar normal.</param>
        /// <returns>A vector</returns>
        public static Vector2 Cross(this Vector2 planar, float normal)
            => new Vector2(planar.Y * normal, -planar.X * normal);

        /// <summary>
        /// Vector/scalar cross product.
        /// <code>ω × [x,y] = [-y*ω, x*ω]</code>
        /// </summary>
        /// <param name="normal">The scalar normal</param>
        /// <param name="planar">The planar vector</param>
        /// <returns>A vector</returns>
        public static Vector2 Cross(this float normal, Vector2 planar)
            => -Cross(planar, normal);
        #endregion

        #region Spatial

        public static Vector3 Sum(this Vector3[] vectors)
        {
            var sum = Vector3.Zero;
            for (int i = 0; i < vectors.Length; i++)
            {
                sum += vectors[i];
            }
            return sum;
        }
        public static Vector3 Average(this Vector3[] vectors)
            => vectors.Sum() / vectors.Length;

        public static Vector3 GetNormal(Vector3 A, Vector3 B, Vector3 C)
        {
            return Vector3.Normalize(
                Vector3.Cross(A, B)
                + Vector3.Cross(B, C)
                + Vector3.Cross(C, A)
                );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Unit(this Vector3 vector)
            => Vector3.Zero == vector ? Vector3.Zero : Vector3.Normalize(vector);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 RandomVector(float minValue = 0, float maxValue = 1)
        {
            return new Vector3(
                minValue + (float)((maxValue - minValue) * RandomNumberGenerator.NextDouble()),
                minValue + (float)((maxValue - minValue) * RandomNumberGenerator.NextDouble()),
                minValue + (float)((maxValue - minValue) * RandomNumberGenerator.NextDouble()));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Rotate(this Vector3 vector, Quaternion rotation)
            => Vector3.Transform(vector, rotation);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 RotateX(this Vector3 vector, float angle)
            => Vector3.Transform(vector, Matrix4x4.CreateRotationX(angle));
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 RotateY(this Vector3 vector, float angle)
            => Vector3.Transform(vector, Matrix4x4.CreateRotationY(angle));
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 RotateZ(this Vector3 vector, float angle)
            => Vector3.Transform(vector, Matrix4x4.CreateRotationZ(angle));
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(this Vector3 vector, Vector3 other)
            => Vector3.Dot(vector, other);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Outer(this Vector3 vector, Vector3 other)
            => new Matrix3(
                vector.X * other.X, vector.X * other.Y, vector.X * other.Z,
                vector.Y * other.X, vector.Y * other.Y, vector.Y * other.Z,
                vector.Z * other.X, vector.Z * other.Y, vector.Z * other.Z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 Cross(this Vector3 vector, Vector3 other)
            => Vector3.Cross(vector, other);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix3 Cross(this Vector3 vector)
            => Matrix3.SkewSymmetric(-vector.Z, vector.Y, -vector.X);

        public static Matrix3 Mmoi(this Vector3 position)
            => Dot(position, position) - Outer(position, position);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float[] ToArray(this Vector3 vector)
        {
            return new[] { vector.X, vector.Y, vector.Z };
        }

        public static Matrix3 Rotation(this Matrix4x4 matrix)
        {
            return new Matrix3(
                matrix.M11, matrix.M21, matrix.M31,
                matrix.M12, matrix.M22, matrix.M32,
                matrix.M13, matrix.M23, matrix.M33);
        }

        public static Matrix4x4 Rotation(this Matrix3 matrix)
        {
            var (m_11, m_12, m_13, m_21, m_22, m_23, m_31, m_32, m_33) = matrix.data;
            return new Matrix4x4(
                m_11, m_21, m_31, 0,
                m_12, m_22, m_32, 0,
                m_13, m_23, m_33, 0,
                0, 0, 0, 1);
        }

        public static Quaternion Derivative(this Quaternion q, Vector3 omega)
        {
            var q_w = new Quaternion(0.5f*omega, 0);
            return Quaternion.Multiply(q_w, q);
        }

        #endregion

    }
    public class AbsFloatComparer :
        IComparer<float>,
        IEqualityComparer<float>,
        System.Collections.IComparer,
        System.Collections.IEqualityComparer
    {
        public AbsFloatComparer(float delta)
        {
            Delta = delta;
        }

        public float Delta { get; }

        public int Compare(float x, float y)
        {
            if (Math.Abs(x - y) <= Delta) return 0;
            if (x > y) return 1;
            return -1;
        }

        public bool Equals(float x, float y)
        {
            return Compare(x, y) == 0;
        }

        public int GetHashCode(float obj)
        {
            return obj.GetHashCode();
        }

        int System.Collections.IComparer.Compare(object x_obj, object y_obj)
        {
            if (x_obj is float x && y_obj is float y)
            {
                return Compare(x, y);
            }
            return -1;
        }

        bool System.Collections.IEqualityComparer.Equals(object x_obj, object y_obj)
        {
            if (x_obj is float x && y_obj is float y)
            {
                return Equals(x, y);
            }
            return false;
        }

        int System.Collections.IEqualityComparer.GetHashCode(object obj)
        {
            if (obj is float x)
            {
                return x.GetHashCode();
            }
            return obj.GetHashCode();
        }
    }

}
