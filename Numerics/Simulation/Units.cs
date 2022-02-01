using System;
using System.Collections.Generic;
using System.Linq;

namespace JA.Numerics.Simulation
{
    public enum UnitSystem
    {
        SI,
        MMKS,
        IPS,
        FPS,
    }
    public interface ICanConvert<out TItem>
    {
        TItem ConvertFromTo(UnitSystem unit, UnitSystem target);
    }
    public interface IHasUnits<out TItem>
    {
        UnitSystem Units { get; }
        TItem ConvertTo(UnitSystem target);
    }

    public enum UnitType
    {
        None,
        Length,
        Mass,
        Force,
        Time,
        Temperature,
    }

    public static class UnitFactors
    {
        public static float Convert(UnitType type, UnitSystem from, UnitSystem to)
        {
            return Factor(type, from)/Factor(type, to);
        }
        public static float Factor(UnitType type, UnitSystem units)
        {
            switch (type)
            {
                case UnitType.None:
                    return 1;
                case UnitType.Length:
                    return Length(units);
                case UnitType.Mass:
                    return Mass(units);
                case UnitType.Force:
                    return Force(units);
                case UnitType.Time:
                    return 1;
                case UnitType.Temperature:
                    return Temperature(units);
                default:
                    throw new NotSupportedException(type.ToString());
            }
        }

        public static float Length(UnitSystem units)
        {
            switch (units)
            {
                case UnitSystem.SI:
                    return 1f;
                case UnitSystem.MMKS:
                    return 0.001f;
                case UnitSystem.IPS:
                    return 0.0254f;
                case UnitSystem.FPS:
                    return 12 * 0.0254f;
                default:
                    throw new NotSupportedException(units.ToString());
            }
        }
        public static float Length(UnitSystem from, UnitSystem to)
        {
            return Length(from) / Length(to);
        }
        public static float Mass(UnitSystem units)
        {
            switch (units)
            {
                case UnitSystem.SI:
                case UnitSystem.MMKS:
                    return 1f;
                case UnitSystem.IPS:
                case UnitSystem.FPS:
                    return 0.4535924f;
                default:
                    throw new NotSupportedException(units.ToString());
            }
        }
        public static float Mass(UnitSystem from, UnitSystem to)
        {
            return Mass(from) / Mass(to);
        }
        public static float Force(UnitSystem units)
        {
            switch (units)
            {
                case UnitSystem.SI:
                case UnitSystem.MMKS:
                    return 1f;
                case UnitSystem.IPS:
                case UnitSystem.FPS:
                    return 4.4482216f;
                default:
                    throw new NotSupportedException(units.ToString());
            }
        }
        public static float Force(UnitSystem from, UnitSystem to)
        {
            return Force(from) / Force(to);
        }
        public static float Torque(UnitSystem units)
            => Force(units) * Length(units);
        public static float Torque(UnitSystem from, UnitSystem to)
            => Force(from, to) * Length(from, to);
        public static float Work(UnitSystem units)
            => Force(units) * Length(units);
        public static float Work(UnitSystem from, UnitSystem to)
            => Force(from, to) * Length(from, to);
        public static float Momentum(UnitSystem units)
            => Mass(units) * Length(units);
        public static float Momentum(UnitSystem from, UnitSystem to)
            => Mass(from, to) * Length(from, to);
        public static float AngularMomentum(UnitSystem units)
            => Mass(units) * Length(units) * Length(units);
        public static float AngularMomentum(UnitSystem from, UnitSystem to)
            => Mass(from, to) * Length(from, to) * Length(from, to);

        public static float Temperature(UnitSystem units)
        {
            switch (units)
            {
                case UnitSystem.SI:
                case UnitSystem.MMKS:
                    return 1f;
                case UnitSystem.IPS:
                case UnitSystem.FPS:
                    return 1/1.8f;
                default:
                    throw new NotSupportedException(units.ToString());
            }
        }
        public static float Temperature(UnitSystem from, UnitSystem to)
        {
            return Temperature(from) / Temperature(to);
        }

        public static float Area(UnitSystem units)
        {
            return Length(units) * Length(units);
        }
        public static float Area(UnitSystem from, UnitSystem to)
        {
            return Area(from) / Area(to);
        }
        public static float Volume(UnitSystem units)
        {
            return Length(units) * Length(units) *Length(units);
        }
        public static float Volume(UnitSystem from, UnitSystem to)
        {
            return Volume(from) / Volume(to);
        }
        public static float Density(UnitSystem units)
        {
            return Mass(units) / Volume(units);
        }
        public static float Density(UnitSystem from, UnitSystem to)
        {
            return Density(from) / Density(to);
        }
        public static float Pressure(UnitSystem units)
        {
            return Force(units) / Area(units);
        }
        public static float Pressure(UnitSystem from, UnitSystem to)
        {
            return Pressure(from) / Pressure(to);
        }
    }

    public abstract class Unit : IEquatable<Unit>
    {
        public abstract float Factor(UnitSystem units);
        public abstract UnitType GetBase();
        public abstract int Complexity { get; }

        public float Convert(UnitSystem from, UnitSystem to)
        {
            return Factor(from)/Factor(to);
        }

        #region Common Units
        public static readonly Unit None = new NoUnit();
        public static readonly Unit Length = new BaseUnit(UnitType.Length);
        public static readonly Unit Mass = new BaseUnit(UnitType.Mass);
        public static readonly Unit Force = new BaseUnit(UnitType.Force);
        public static readonly Unit Time = new BaseUnit(UnitType.Time);
        public static readonly Unit Temperature = new BaseUnit(UnitType.Temperature);
        public static readonly Unit Velocity = Length/Time;
        public static readonly Unit Acceleration = Velocity/Time;
        public static readonly Unit Momentum = Mass*Velocity;
        public static readonly Unit Impulse = Force * Time;
        public static readonly Unit PerTemperature = 1/Temperature;
        public static readonly Unit Area = Length ^ 2;
        public static readonly Unit Volume = Length ^ 3;
        public static readonly Unit Density = Mass/Volume;
        #endregion

        #region Factory
        public static Unit Base(UnitType type)
        {
            if (type == UnitType.None) return None;
            return new BaseUnit(type);
        }

        public static Unit Combine(Unit a, Unit b)
        {
            if (a.IsScalar && b.IsScalar) return None;
            if (a.IsScalar) return b;
            if (b.IsScalar) return a;
            if (a.Equals(b)) return Raise(a, 2);
            if (a.IsDerived(out var s1, out var n1, out var u1) && b.IsDerived(out var s2, out var n2, out var u2))
            {
                if (u1.Equals(u2))
                {
                    return Derived(s1*s2, Raise(u1, n1+n2));
                }
            }
            if (a is CombineUnit cu1 && b is CombineUnit cu2)
            {
                Unit[] parts = new Unit[cu1.Count + cu2.Count];
                Array.Copy(cu1.Parts, 0, parts, 0, cu1.Count);
                Array.Copy(cu2.Parts, 0, parts, cu1.Count, cu2.Count);
                return Combine(parts);
            }
            if (a is CombineUnit cu3)
            {
                Unit[] parts = new Unit[cu3.Count + 1];
                Array.Copy(cu3.Parts, 0, parts, 0, cu3.Count);
                parts[cu3.Count] = b;
                return Combine(parts);
            }
            if (b is CombineUnit)
            {
                Combine(b, a);
            }
            return new CombineUnit(a, b);
        }
        public static Unit Combine(params Unit[] parts)
        {
            parts = parts.Where((u) => !u.IsScalar).ToArray();
            if (parts.Length == 0) return None;
            if (parts.Length == 1) return parts[0];
            return new CombineUnit(parts);
        }

        public static Unit Derived(float scale, Unit unit)
        {
            if (scale==0) return None;
            if (scale==1) return unit;
            if (unit is NoUnit) return None;
            if (unit is DerivedUnit du)
            {
                return Derived(scale*du.Scale, du.Argument);
            }
            return new DerivedUnit(scale, unit);
        }

        public static Unit Raise(Unit unit, int exponent)
        {
            if (exponent == 0) return None;
            if (exponent==1) return unit;
            if (unit is RaiseUnit ru)
            {
                return Raise(ru.Argument, exponent*ru.Exponent);
            }
            if (unit is DerivedUnit du)
            {
                return Derived((float)Math.Pow(du.Scale, exponent), Raise(du.Argument, exponent));
            }
            if (unit is CombineUnit cu)
            {
                return Combine(cu.Parts.Select((u) => Raise(u, exponent)).ToArray());
            }
            return new RaiseUnit(unit, exponent);
        }
        #endregion

        public static implicit operator Unit(float factor) => Derived(factor, None);
        public static implicit operator Unit(UnitType type) => Base(type);

        #region Properties
        public bool IsScalar { get => this is NoUnit || this is BaseUnit bu && bu.Type==UnitType.None; }
        public bool IsBase { get => this is BaseUnit bu && bu.Type != UnitType.None; }

        public bool IsSameType(Unit other)
        {
            if (Equals(other)) return true;
            if (IsDerived(out var s1, out var n1, out var u1) && other.IsDerived(out var s2, out var n2, out var u2))
            {
                return n1==n2 && u1.Equals(u2);
            }
            return GetBase() == other.GetBase();
        }
        public bool IsDerived(out float scale, out int exponent, out Unit argument)
        {
            if (this is NoUnit nu)
            {
                scale = 1;
                exponent = 1;
                argument = None;
                return true;
            }
            else if (this is BaseUnit bu)
            {
                scale = 1;
                exponent = 1;
                argument = bu;
                return true;
            }
            else if (this is DerivedUnit du)
            {
                scale = du.Scale;
                if (du.Argument is RaiseUnit ru)
                {
                    exponent = ru.Exponent;
                    argument = ru.Argument;
                    return ru.IsBase;
                }
                exponent = 1;
                argument = du.Argument;
                return du.IsBase;
            }
            else if (this is RaiseUnit ru)
            {
                scale = 1;
                exponent = ru.Exponent;
                argument = ru.Argument;
                return ru.IsBase;
            }
            scale = 1;
            exponent = 0;
            argument = None;
            return false;
        }

        public static int Compare(Unit a, Unit b)
        {
            var eq = a.Complexity.CompareTo(b.Complexity);
            if (eq == 0)
            {
                eq = a.GetBase().CompareTo(b.GetBase());
            }
            return eq;
        }
        #endregion

        #region Operators
        public static Unit operator *(float scale, Unit a) => Derived(scale, a);
        public static Unit operator *(Unit a, float scale) => Derived(scale, a);
        public static Unit operator /(Unit a, float divisor) => Derived(1/divisor, a);
        public static Unit operator *(Unit a, Unit b) => Combine(a, b);
        public static Unit operator /(Unit a, Unit b) => Combine(a, Raise(b, -1));
        public static Unit operator ^(Unit a, int exponent) => Raise(a, exponent);
        public static float operator |(Unit a, UnitSystem units) => a.Factor(units);
        #endregion

        #region Unit Nodes
        class NoUnit : Unit
        {
            public override float Factor(UnitSystem units) => 1;
            public override UnitType GetBase() => UnitType.None;
            public override int Complexity => 0;
            public override bool Equals(Unit other)
            {
                if (other is NoUnit)
                {
                    return true;
                }
                return false;
            }
            public override int GetHashCode()
            {
                return -1817952719;
            }
        }
        class BaseUnit : Unit
        {
            public BaseUnit(UnitType type)
            {
                Type=type;
            }
            public UnitType Type { get; }
            public override UnitType GetBase() => Type;
            public override int Complexity => 1;
            public override float Factor(UnitSystem units)
            {
                return UnitFactors.Factor(Type, units);
            }
            public override int GetHashCode()
            {
                return Type.GetHashCode();
            }
            public override bool Equals(Unit other)
            {
                if (other is BaseUnit bu)
                {
                    return Type == bu.Type;
                }
                return false;
            }
        }
        class DerivedUnit : Unit
        {
            public DerivedUnit(float scale, Unit argument)
            {
                Scale=scale;
                Argument=argument;
            }
            public float Scale { get; }
            public Unit Argument { get; }
            public override UnitType GetBase() => Argument.GetBase();
            public override int Complexity => 1 + Argument.Complexity;
            public override float Factor(UnitSystem units)
            {
                return Scale * Argument.Factor(units);
            }
            public override bool Equals(Unit other)
            {
                if (other is DerivedUnit du)
                {
                    return Scale == du.Scale
                        && Argument.Equals(du.Argument);
                }
                return false;
            }
            public override int GetHashCode()
            {
                unchecked
                {
                    int hc = -1817952719;
                    hc = (-1521134295)*hc + Scale.GetHashCode();
                    hc = (-1521134295)*hc + Argument.GetHashCode();
                    return hc;
                }
            }
        }
        class RaiseUnit : Unit
        {
            public RaiseUnit(Unit argument, int exponent)
            {
                Argument=argument;
                Exponent=exponent;
            }
            public Unit Argument { get; }
            public int Exponent { get; }
            public override int Complexity => 1 + Argument.Complexity;
            public override UnitType GetBase() => Argument.GetBase();
            public override float Factor(UnitSystem units)
            {
                var f = Argument.Factor(units);
                switch (Exponent)
                {
                    case 0: return 1;
                    case 1: return f;
                    case -1: return 1/f;
                    case 2: return f*f;
                    case 3: return f*f*f;
                    case -2: return 1/(f*f);
                    case -3: return 1/(f*f*f);
                    default:
                        return (float)Math.Pow(f, Exponent);
                }
            }
            public override bool Equals(Unit other)
            {
                if (other is RaiseUnit ru)
                {
                    return Exponent == ru.Exponent
                        && Argument.Equals(ru.Argument);
                }
                return false;
            }
            public override int GetHashCode()
            {
                unchecked
                {
                    int hc = -1817952719;
                    hc = (-1521134295)*hc + Exponent.GetHashCode();
                    hc = (-1521134295)*hc + Argument.GetHashCode();
                    return hc;
                }
            }
        }
        class CombineUnit : Unit
        {
            public CombineUnit(params Unit[] parts)
            {
                Parts=parts;
                Array.Sort(Parts, (x, y) => Compare(x, y));
            }
            public int Count { get => Parts.Length; }
            public Unit[] Parts { get; }
            public override UnitType GetBase() => Parts.Length==1 ? Parts[0].GetBase() : UnitType.None;
            public override int Complexity => 1 + Parts.Max((p) => p.Complexity);
            public override float Factor(UnitSystem units)
            {
                float f = 1;
                for (int i = 0; i < Parts.Length; i++)
                {
                    f *= Parts[i].Factor(units);
                }
                return f;
            }

            public override bool Equals(Unit other)
            {
                if (other is CombineUnit cu)
                {
                    return Enumerable.SequenceEqual(Parts, cu.Parts);
                }
                return false;
            }
            public override int GetHashCode()
            {
                int hc = -1817952719;
                for (int i = 0; i < Parts.Length; i++)
                {
                    hc = (-1521134295)*hc + Parts[i].GetHashCode();
                }
                return hc;
            }
        }
        #endregion

        #region IEquatable Members
        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Unit)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Unit other)
            {
                return Equals(other);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among <see cref="Unit"/> classes
        /// </summary>
        /// <param name="other">The other <see cref="Unit"/> to compare it to</param>
        /// <returns>True if equal</returns>
        public abstract bool Equals(Unit other);

        /// <summary>
        /// Calculates the hash code for the <see cref="Unit"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override abstract int GetHashCode();

        #endregion

    }

}
