using System;
using System.Runtime.CompilerServices;

namespace JA.Numerics
{
    public class Vector<TData> :IEquatable<Vector<TData>>
    {
        protected internal TData[] Data { get; }

        public Vector(int size)
        {
            Data = new TData[size];
        }
        protected Vector(int size, TData[] values) : this(size)
        {
            Array.Copy(values, Data, size);
        }
        public Vector(int size, DataFunction<TData> init)
            : this(size)
        {
            for (int index = 0; index < Data.Length; index++)
            {
                Data[index] = init(index);
            }
        }
        public int Size { get => Data.Length; }
        public int Count { get => Data.Length; }

        protected internal ref TData this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => ref Data[index];
        }
        public TData[] ToArray() => Data;
        public ReadOnlySpan<TData> AsSpan() => Data.AsSpan();
        public ReadOnlySpan<TData> AsSpan(int start, int length) => Data.AsSpan(start, length);
        #region IEquatable Members

        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Vector)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Vector<TData> vector)
            {
                return Equals(vector);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among <see cref="Vector{TData}"/> classes
        /// </summary>
        /// <returns>True if equal</returns>
        public bool Equals(Vector<TData> other)
        {
            return LinearAlgebra.Equals(Data, other.Data);
        }
        /// <summary>
        /// Calculates the hash code for the <see cref="Vector{TData}"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            return LinearAlgebra.GetHashCode(Data);
        }

        #endregion


    }

    public sealed class Vector : Vector<float>, IEquatable<Vector>
    {
        public Vector(int size) : base(size) { }
        Vector(int size, float[] data) : base(size, data) { }
        public Vector(int size, DataFunction<float> init) : base(size, init) { }
        public static Vector FromArray(params float[] values)
            => new Vector(values.Length, values);
        public static Vector Zero(int size) => new Vector(size);

        #region Algebra
        public static Vector Negate(Vector a)
        {
            var result = new float[a.Size];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = -a[i];
            }
            return Vector.FromArray(result);
        }

        public static Vector Scale(float factor, Vector a)
        {
            var result = new float[a.Size];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = factor * a[i];
            }
            return Vector.FromArray(result);
        }

        public static Vector Add(Vector a, Vector b)
        {
            if (a.Size != b.Size) throw new ArgumentException(nameof(b));
            var result = new float[a.Size];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = a[i] + b[i];
            }
            return Vector.FromArray(result);
        }

        public static Vector Subtract(Vector a, Vector b)
        {
            if (a.Size != b.Size) throw new ArgumentException(nameof(b));
            var result = new float[a.Size];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = a[i] - b[i];
            }
            return Vector.FromArray(result);
        }

        public static float Dot(Vector a, Vector b)
        {
            if (a.Size != b.Size) throw new ArgumentException(nameof(b));
            float result = 0;
            for (var index = 0; index < a.Size; index++)
            {
                result += a[index] * b[index];
            }            
            return result;
        }
        public static Matrix Outer(Vector a, Vector b)
        {
            if (a.Size != b.Size) throw new ArgumentException(nameof(b));
            int n = a.Size;
            if (a.Equals(b))
            {
                float[] results = new float[MatrixShape.GetDataSize(MatrixStructure.Symmetric, n)];
                int index = 0;
                for (int row = 0; row < n; row++)
                {
                    for (int column = row; column < n; column++)
                    {
                        results[index++] = a[row] * b[column];
                    }
                }
                return Matrix.Symmetric(results);
            }
            else
            {
                float[] results = new float[MatrixShape.GetDataSize(MatrixStructure.Dense, n)];
                int index = 0;
                for (int row = 0; row < n; row++)
                {
                    for (int column = 0; column < n; column++)
                    {
                        results[index++] = a[row] * b[column];
                    }
                }
                return Matrix.Dense(results);
            }
        }
        public static Vector operator +(Vector a, Vector b) => Add(a, b);
        public static Vector operator -(Vector a) => Negate(a);
        public static Vector operator -(Vector a, Vector b) => Subtract(a, b);
        public static Vector operator *(float f, Vector a) => Scale(f, a);
        public static Vector operator *(Vector a, float f) => Scale(f, a);
        public static Vector operator /(Vector a, float d) => Scale(1 / d, a);

        public static float operator *(Vector a, Vector b) => Dot(a, b);
        public static Matrix operator %(Vector a, Vector b) => Outer(a, b);
        #endregion

        public bool Equals(Vector other) => base.Equals(other);
    }
}
