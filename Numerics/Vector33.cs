using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using JA.Numerics.Simulation;

namespace JA.Numerics
{
    public enum ScrewType
    {
        Twist,
        Wrench
    }

    [TypeConverter(typeof(ExpandableObjectConverter))]
    public struct Vector33 : IEquatable<Vector33>
    {
        internal readonly (Vector3 m_1, Vector3 m_2) data;

        #region Factory
        public Vector33(Vector3 m_1, Vector3 m_2)
            : this((m_1, m_2)) { }
        public Vector33((Vector3 m_1, Vector3 m_2) data) : this()
        {
            this.data = data;
        }

        public static readonly Vector33 Zero = new Vector33(Vector3.Zero, Vector3.Zero);
        public static Vector33 Twist(Vector3 moment, Vector3 value, Vector3 position)
            => new Vector33(moment + LinearAlgebra.Cross(position,value), value);
        public static Vector33 Twist(Vector3 value, Vector3 position, float pitch)
            => new Vector33(LinearAlgebra.Cross(position, value) + pitch * value, value);
        public static Vector33 Twist(Vector3 value)
            => new Vector33(value, Vector3.Zero);
        public static Vector33 Wrench(Vector3 moment, Vector3 value, Vector3 position)
            => new Vector33(value, moment + LinearAlgebra.Cross(position, value));
        public static Vector33 Wrench(Vector3 value, Vector3 position, float pitch)
            => new Vector33(value, LinearAlgebra.Cross(position, value) + pitch * value);
        public static Vector33 Wrench(Vector3 value)
            => new Vector33(Vector3.Zero, value);

        public static Vector3 TwistValue(Vector33 twist, Vector3 position)
            => twist.Vector1 + LinearAlgebra.Cross(twist.Vector2, position);

        public static Vector3 WrenchValue(Vector33 wrench, Vector3 position)
            => wrench.Vector2 + LinearAlgebra.Cross(wrench.Vector1, position);
        #endregion

        #region Properties
        public (Vector3 a1, Vector3 a2) Data => data;
        [Category("Data")]
        public Vector3 Vector1 { get => data.m_1; }
        [Category("Data")]
        public Vector3 Vector2 { get => data.m_2; }
        [Browsable(false)]
        public bool IsZero => data.m_1 == Vector3.Zero && data.m_2 == Vector3.Zero;

        public Vector3 TwistAt(Vector3 position)
            => data.m_1- LinearAlgebra.Cross(position, data.m_2);
        public Vector3 WrenchAt(Vector3 position)
            => data.m_2 - LinearAlgebra.Cross(position, data.m_1);

        [Category("Twist")]
        public float TwistMagnitude => data.m_2.Length();
        [Category("Twist")]
        public Vector3 TwistDirection => Vector3.Normalize(data.m_2);
        [Category("Twist")]
        public Vector3 TwistCenter
            => LinearAlgebra.Cross(data.m_2, data.m_1) / data.m_2.LengthSquared();

        [Category("Wrench")]
        public float WrenchMagnitude => data.m_1.Length();
        [Category("Wrench")]
        public Vector3 WrenchDirection => Vector3.Normalize(data.m_1);
        [Category("Wrench")]
        public Vector3 WrenchCenter
            => LinearAlgebra.Cross(data.m_1, data.m_2) / data.m_1.LengthSquared();
        #endregion

        #region Algebra
        public static Vector33 Negate(Vector33 A)
            => new Vector33(
                -A.data.m_1,
                -A.data.m_2);
        public static Vector33 Add(Vector33 A, Vector33 B) 
            => new Vector33(
                A.data.m_1 + B.data.m_1,
                A.data.m_2 + B.data.m_2);
        public static Vector33 Subtract(Vector33 A, Vector33 B)
            => new Vector33(
                A.data.m_1 - B.data.m_1,
                A.data.m_2 - B.data.m_2);

        public static Vector33 Scale(float factor, Vector33 A)
            => new Vector33(
                factor * A.data.m_1,
                factor * A.data.m_2);

        public static float Dot(Vector33 A, Vector33 B) 
            => Vector3.Dot(A.data.m_1, B.data.m_1)
                + Vector3.Dot(A.data.m_2, B.data.m_2);

        public static Matrix33 Outer(Vector33 A, Vector33 B)
            => new Matrix33(
                A.data.m_1.Outer(B.data.m_1), A.data.m_1.Outer(B.data.m_2),
                A.data.m_2.Outer(B.data.m_1), A.data.m_2.Outer(B.data.m_2));

        public static Vector33 CrossTwistTwist(Vector33 A, Vector33 B)
            => new Vector33(
                Vector3.Cross(A.Vector2, B.Vector1) + Vector3.Cross(A.Vector1, B.Vector2),
                Vector3.Cross(A.Vector2, B.Vector2));
        public static Vector33 CrossTwistWrench(Vector33 A, Vector33 B)
            => new Vector33(
                Vector3.Cross(A.Vector2, B.Vector1),
                Vector3.Cross(A.Vector1, B.Vector1) + Vector3.Cross(A.Vector2, B.Vector2));
        public static Vector33 CrossWrenchTwist(Vector33 A, Vector33 B)
            => new Vector33(
                Vector3.Cross(A.Vector1, B.Vector2),
                Vector3.Cross(A.Vector1, B.Vector1) + Vector3.Cross(A.Vector2, B.Vector2));
        public static Vector33 CrossWrenchWrench(Vector33 A, Vector33 B)
            => new Vector33(
                Vector3.Cross(A.Vector1, B.Vector2) + Vector3.Cross(A.Vector2, B.Vector1),
                Vector3.Cross(A.Vector1, B.Vector1));
        #endregion

        #region Operators
        public static Vector33 operator +(Vector33 a, Vector33 b) => Add(a, b);
        public static Vector33 operator -(Vector33 a) => Scale(-1, a);
        public static Vector33 operator -(Vector33 a, Vector33 b) => Subtract(a, b);
        public static Vector33 operator *(float a, Vector33 b) => Scale(a, b);
        public static Vector33 operator *(Vector33 a, float b) => Scale(b, a);
        public static Vector33 operator /(Vector33 a, float b) => Scale(1 / b, a);
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Vector33)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Vector33 other)
            {
                return Equals(other);
            }
            return false;
        }

        public static bool operator ==(Vector33 target, Vector33 other) { return target.Equals(other); }
        public static bool operator !=(Vector33 target, Vector33 other) { return !(target == other); }


        /// <summary>
        /// Checks for equality among <see cref="Vector33"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Vector33"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public bool Equals(Vector33 other)
        {
            return data.Equals(other.data);
        }

        /// <summary>
        /// Calculates the hash code for the <see cref="Vector33"/>
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
        [Browsable(false)] public int Count => 2;

        /// <summary>
        /// Get the elemetns as an array
        /// </summary>
        public Vector3[] ToArray() => AsSpan().ToArray();

        /// <summary>
        /// Get the elements as a span
        /// </summary>
        public unsafe ReadOnlySpan<Vector3> AsSpan()
        {
            fixed (Vector3* ptr = &data.m_1)
            {
                return new ReadOnlySpan<Vector3>(ptr, Count);
            }
        }

        #endregion

        #region Formatting
        public override string ToString() => $"Vector33(" +
            $"{data.m_1.X:g3},{data.m_1.Y:g3},{data.m_1.Z:g3} | " +
            $"{data.m_2.X:g3},{data.m_2.Y:g3},{data.m_2.Z:g3})";
        #endregion

        public Vector33 ConvertFromTo(UnitSystem units, UnitSystem target, UnitType valueUnit)
        {
            switch (valueUnit)
            {
                case UnitType.Length:
                    return ConvertFromTo(units, target, valueUnit, ScrewType.Twist);
                case UnitType.Mass:
                case UnitType.Force:
                    return ConvertFromTo(units, target, valueUnit, ScrewType.Wrench);
                case UnitType.Temperature:
                case UnitType.None:
                case UnitType.Time:
                default:
                    throw new NotSupportedException(valueUnit.ToString());
            }
        }
        public Vector33 ConvertFromTo(UnitSystem units, UnitSystem target, UnitType valueUnit, ScrewType type)
            => ConvertFromTo(units, target, (Unit)valueUnit, type);
        public Vector33 ConvertFromTo(UnitSystem units, UnitSystem target, Unit valueUnit, ScrewType type)
        {
            if (units == target) return this;
            var fl = Unit.Length.Convert(units, target);
            var ff = valueUnit.Convert(units, target);
            switch (type)
            {
                case ScrewType.Twist:
                    return new Vector33(fl*ff*data.m_1, ff*data.m_2);
                case ScrewType.Wrench:
                    return new Vector33(ff*data.m_1, fl*ff*data.m_2);
                default:
                    throw new NotSupportedException(type.ToString());
            }
        }

    }
}
