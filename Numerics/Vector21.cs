using System;
using System.Numerics;

namespace JA.Numerics
{
    public struct Vector21 : IEquatable<Vector21>
    {
        internal readonly (Vector2 m_1, float m_2) data;

        #region Factory
        public Vector21(Vector2 m_1, float m_2) : this((m_1, m_2)) { }
        public Vector21((Vector2 m_1, float m_2) data)
        {
            this.data = data;
        }
        public static readonly Vector21 Zero = new Vector21((Vector2.Zero, 0));
        public static readonly Vector21 AlongX = new Vector21((Vector2.UnitX, 0));
        public static readonly Vector21 AlongY = new Vector21((Vector2.UnitY, 0));
        public static readonly Vector21 AboutZ = new Vector21((Vector2.Zero, 1));

        public static Vector21 Twist(float value, Vector2 position)
            => new Vector21(LinearAlgebra.Cross(position, value), value);
        public static Vector21 Twist(Vector2 value)
            => new Vector21(value, 0);
        public static Vector21 Wrench(Vector2 value, Vector2 position)
            => new Vector21(value, LinearAlgebra.Cross(position, value));
        public static Vector21 Wrench(float value)
            => new Vector21(Vector2.Zero, value);
        #endregion

        #region Properties
        public (Vector2 m_1, float m_2) Data => data;

        public Vector2 Vector => data.m_1;
        public float Scalar => data.m_2;

        public bool IsZero { get => data.m_2 == 0 && data.m_1 == Vector2.Zero; }

        public Vector2 TwistAt(Vector2 position)
            => data.m_1 - LinearAlgebra.Cross(position, data.m_2);
        public float WrenchAt(Vector2 position)
            => data.m_2 - LinearAlgebra.Cross(position, data.m_1);

        public float TwistMagnitude => Math.Abs(data.m_2);
        public int TwistDirection => Math.Sign(data.m_2);
        public float WrenchMagnitude => data.m_1.Length();
        public Vector2 WrenchDirection => Vector2.Normalize(data.m_1);

        public Vector2 TwistCenter
            => LinearAlgebra.Cross(data.m_2, data.m_1) / (data.m_2 * data.m_2);

        public Vector2 WrenchCenter
            => LinearAlgebra.Cross(data.m_1, data.m_2) / data.m_1.LengthSquared();

        #endregion

        #region Algebra
        public static Vector21 Negate(Vector21 twist)
            => new Vector21(
                -twist.data.m_1,
                -twist.data.m_2);
        public static Vector21 Scale(float factor, Vector21 twist)
            => new Vector21(
                factor * twist.data.m_1,
                factor * twist.data.m_2);
        public static Vector21 Add(Vector21 a, Vector21 b)
            => new Vector21(
                a.data.m_1 + b.data.m_1,
                a.data.m_2 + b.data.m_2);
        public static Vector21 Subtract(Vector21 a, Vector21 b)
            => new Vector21(
                a.data.m_1 - b.data.m_1,
                a.data.m_2 - b.data.m_2);
        public static float Dot(Vector21 a, Vector21 b)
            => Vector2.Dot(a.data.m_1, b.data.m_1)
            + a.data.m_2 * b.data.m_2;

        public static Matrix21 Outer(Vector21 a, Vector21 b)
            => new Matrix21(
                LinearAlgebra.Outer(a.data.m_1, b.data.m_1),
                a.data.m_1 * b.data.m_2,
                a.data.m_2 * b.data.m_1,
                a.data.m_2 * b.data.m_2);

        public static Vector21 CrossTwistTwist(Vector21 a, Vector21 b)
            => new Vector21(
                a.data.m_2.Cross(b.data.m_1) + a.data.m_1.Cross(b.data.m_2),
                0);

        public static Vector21 CrossTwistWrench(Vector21 a, Vector21 b)
            => new Vector21(
                a.data.m_2.Cross(b.data.m_1),
                a.data.m_1.Cross(b.data.m_1));

        public static Vector21 CrossWrenchWrench(Vector21 a, Vector21 b)
            => new Vector21(
                a.data.m_1.Cross(b.data.m_2) + a.data.m_2.Cross(b.data.m_1),
                a.data.m_1.Cross(b.data.m_1));

        public static Vector21 CrossWrenchTwist(Vector21 a, Vector21 b)
            => new Vector21(
                a.data.m_1.Cross(b.data.m_2),
                a.data.m_1.Cross(b.data.m_1));


        public static Vector21 operator -(Vector21 a) => Negate(a);
        public static Vector21 operator +(Vector21 a, Vector21 b) => Add(a, b);
        public static Vector21 operator -(Vector21 a, Vector21 b) => Subtract(a, b);
        public static Vector21 operator *(float f, Vector21 a) => Scale(f, a);
        public static Vector21 operator *(Vector21 a, float f) => Scale(f, a);
        public static Vector21 operator /(Vector21 a, float d) => Scale(1 / d, a);
        public static float operator *(Vector21 a, Vector21 b) => Dot(a, b);
        #endregion

        #region Collection

        public int Count => 3;
        public float[] ToArray() => AsSpan().ToArray();
        public unsafe ReadOnlySpan<float> AsSpan()
        {
            fixed (float* ptr = &data.m_1.X)
            {
                return new ReadOnlySpan<float>(ptr, Count);
            }
        }

        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Vector21)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Vector21 other)
            {
                return Equals(other);
            }
            return false;
        }

        public static bool operator ==(Vector21 target, Vector21 other) { return target.Equals(other); }
        public static bool operator !=(Vector21 target, Vector21 other) { return !(target == other); }


        /// <summary>
        /// Checks for equality among <see cref="Vector21"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Vector21"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Vector21 other)
        {
            return data.Equals(other.data);
        }

        /// <summary>
        /// Calculates the hash code for the <see cref="Vector21"/>
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

        #region Formatting
        public override string ToString() 
            => $"Vector21({data.m_1.X},{data.m_1.Y}|{data.m_2})";
        #endregion
    }

}


