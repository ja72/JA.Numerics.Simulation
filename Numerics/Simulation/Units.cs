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
    public interface IHasUnits
    {
        UnitSystem Units { get; }
    }
    public interface ICanChangeUnits : IHasUnits
    {
        void ConvertTo(UnitSystem target);
    }
    public interface ICanChangeUnits<T> : IHasUnits
    {
        T ConvertTo(UnitSystem target);
    }
    public interface ICanConvertUnits<T> 
    {
        T ConvertFromTo(UnitSystem units, UnitSystem target);
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

    public abstract class Unit : IEquatable<Unit>
    {
        public abstract override string ToString();
        public abstract float Factor(UnitSystem units);
        public abstract UnitType GetBase();
        public abstract int Complexity { get; }

        public float Convert(UnitSystem from, UnitSystem to)
        {
            return Factor(from)/Factor(to);
        }

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
            return new ComplexUnit(a, b);
        }
        public static Unit Combine(float scale, params Unit[] parts)
        {
            return new ComplexUnit(scale, parts);
        }
        public static Unit Combine(params Unit[] parts)
        {
            return new ComplexUnit(parts);
        }

        public static Unit Derived(float scale, Unit unit)
        {
            if (scale==0) return None;
            if (scale==1) return unit;
            if (unit is NoUnit) return None;
            if (unit is ComplexUnit du)
            {
                return Combine(scale*du.Scale, du.Parts);
            }
            return new ComplexUnit(scale, unit);
        }

        public static Unit Raise(Unit unit, int exponent)
        {
            if (exponent == 0) return None;
            if (exponent==1) return unit;
            if (unit is RaiseUnit ru)
            {
                return Raise(ru.Argument, exponent*ru.Exponent);
            }
            if (unit is ComplexUnit cu)
            {
                return Combine((float)Math.Pow(cu.Scale, exponent),  cu.Parts.Select((u) => Raise(u, exponent)).ToArray());
            }
            return new RaiseUnit(unit, exponent);
        }

        #endregion

        #region Common Units
        public static readonly Unit None = new NoUnit();
        public static readonly Unit Angle = new BaseUnit(UnitType.None);
        public static readonly Unit Length = new BaseUnit(UnitType.Length);
        public static readonly Unit Mass = new BaseUnit(UnitType.Mass);
        public static readonly Unit Force = new BaseUnit(UnitType.Force);
        public static readonly Unit Time = new BaseUnit(UnitType.Time);
        public static readonly Unit Temperature = new BaseUnit(UnitType.Temperature);
        public static readonly Unit Speed = Length/Time;
        public static readonly Unit Acceleration = Speed/Time;
        public static readonly Unit RotationalSpeed = Angle/Time;
        public static readonly Unit RotationalAcceleration = RotationalSpeed/Time;
        public static readonly Unit Torque = Force * Length;
        public static readonly Unit Momentum = Mass*Speed;
        public static readonly Unit Impulse = Force * Time;
        public static readonly Unit Work = Force * Length;
        public static readonly Unit Power = Force * Speed;
        public static readonly Unit PerTemperature = 1/Temperature;
        public static readonly Unit Area = Length ^ 2;
        public static readonly Unit Volume = Length ^ 3;
        public static readonly Unit Density = Mass/Volume;
        public static readonly Unit Pressure = Force/Area;
        public static readonly Unit AreaMoment = Length ^ 4;
        public static readonly Unit MassMomentOfInertia = Mass*Area;
        #endregion

        public static implicit operator Unit(float factor) => Derived(factor, None);
        public static implicit operator Unit(UnitType type) => Base(type);

        #region Properties
        public bool IsScalar { get => this is NoUnit || this is BaseUnit bu && bu.Type==UnitType.None; }
        public bool IsBase { get => this is BaseUnit bu && bu.Type != UnitType.None; }

        public bool IsComplex(out float scale, out Unit[] parts)
        {
            if (this is ComplexUnit cu)
            {
                scale = cu.Scale;
                parts = cu.Parts;
                return true;
            }
            scale = 0;
            parts = Array.Empty<Unit>();
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
            public override string ToString()
            {
                return string.Empty;
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
                return GetFactor(Type, units);
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
                else if (other is RaiseUnit ru)
                {
                    ru.Equals(this);
                }
                else if (other is ComplexUnit du)
                {
                    du.Equals(this);
                }
                return false;
            }
            public override string ToString()
            {
                return Type.ToString();
            }
        }
        class RaiseUnit : Unit
        {
            public RaiseUnit(Unit argument, int exponent)
            {
                Exponent=exponent;
                Argument=argument;
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
                return Exponent == 1
                    && Argument.Equals(other);
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
            public override string ToString()
            {
                return Argument.IsBase ? $"{Argument}^{Exponent}" : $"({Argument})^{Exponent}";
            }
        }

        class ComplexUnit : Unit
        {
            public ComplexUnit(params Unit[] parts)
                : this(1, parts) { }
            public ComplexUnit(float scale, Unit[] parts)
            {
                //Combine all complex parts
                while (parts.Any(u=> u is ComplexUnit))
                {
                    List<Unit> temp = new List<Unit>(parts.Length);
                    for (int i = 0; i < parts.Length; i++)
                    {
                        if (parts[i] is ComplexUnit cu)
                        {
                            scale *= cu.Scale;
                            temp.AddRange(cu.Parts);
                        }
                        else
                        {
                            temp.Add(parts[i]);
                        }
                    }
                    parts = temp.ToArray();
                }
                var units = new Dictionary<UnitType, int>
                {
                    [UnitType.Length] = 0,
                    [UnitType.Mass] = 0,
                    [UnitType.Force] = 0,
                    [UnitType.Time] = 0,
                    [UnitType.Temperature] = 0
                };
                for (int i = 0; i < parts.Length; i++)
                {
                    // Store exponents
                    if (parts[i] is BaseUnit bu)
                    {
                        units[bu.Type] += 1;
                    }
                    else if (parts[i] is RaiseUnit ru)
                    {
                        if (ru.Argument is BaseUnit rbu)
                        {
                            units[rbu.Type] += ru.Exponent;
                        }
                        else
                        {
                            throw new NotSupportedException($"Raised unit to complex base of {ru.Argument.GetType().Name}");
                        }
                    }
                    else
                    {
                        throw new NotSupportedException($"Unexpected unit {parts[i].GetType().Name}");
                    }
                }
                Scale = scale;
                List<Unit> basis = new List<Unit>(5);
                foreach (var unit in units)
                {
                    if (unit.Value!=0)
                    {
                        basis.Add(Raise(Base(unit.Key), unit.Value));
                    }
                }
                Parts = basis.ToArray();
                Array.Sort(Parts, Compare);
            }
            public bool IsDerived { get => Scale!=1 && Parts.Length==1; }
            public float Scale { get; }
            public Unit[] Parts { get; }
            public int Count { get => Parts.Length; }
            public override UnitType GetBase() => Parts.Length==1 ? Parts[0].GetBase() : UnitType.None;
            public override int Complexity => 1 + Parts.Max((p) => p.Complexity);
            public override float Factor(UnitSystem units)
            {
                float f = Scale;
                for (int i = 0; i < Parts.Length; i++)
                {
                    f *= Parts[i].Factor(units);
                }
                return f;
            }

            public override bool Equals(Unit other)
            {
                if (other is ComplexUnit cu)
                {
                    return Scale == cu.Scale
                        && Enumerable.SequenceEqual(Parts, cu.Parts);
                }
                else if (IsScalar)
                {
                    return Scale==1 && Parts[0].Equals(other);
                }
                return false;
            }
            public override int GetHashCode()
            {
                int hc = -1817952719;
                hc = (-1521134295)*hc + Scale.GetHashCode();
                for (int i = 0; i < Parts.Length; i++)
                {
                    hc = (-1521134295)*hc + Parts[i].GetHashCode();
                }
                return hc;
            }
            public override string ToString()
            {
                return Scale!=1 ? $"{Scale}{string.Join<Unit>("·", Parts)}" : string.Join<Unit>("·", Parts);
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

        #region Static Conversions
        public static float GetFactor(UnitType type, UnitSystem units)
        {
            switch (type)
            {
                case UnitType.None:
                    return 1;
                case UnitType.Length:
                    return LengthFactor(units);
                case UnitType.Mass:
                    return MassFactor(units);
                case UnitType.Force:
                    return ForceFactor(units);
                case UnitType.Time:
                    return 1;
                case UnitType.Temperature:
                    return TemperatureFactor(units);
                default:
                    throw new NotSupportedException(type.ToString());
            }
        }
        static float LengthFactor(UnitSystem units)
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
        static float MassFactor(UnitSystem units)
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
        static float ForceFactor(UnitSystem units)
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
        static float TemperatureFactor(UnitSystem units)
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

        #endregion

    }
    public static class UnitSystemExtensions
    {
        public static bool IsConsistent(this UnitSystem units)
        {
            switch (units)
            {
                case UnitSystem.SI:
                    return true;
                case UnitSystem.MMKS:
                case UnitSystem.IPS:
                case UnitSystem.FPS:
                    return false;
                default:
                    throw new NotSupportedException($"Unknown unit system {units}");
            }
        }
        public static bool IsMetric(this UnitSystem units)
        {
            switch (units)
            {
                case UnitSystem.SI:
                case UnitSystem.MMKS:
                    return true;
                case UnitSystem.IPS:
                case UnitSystem.FPS:
                    return false;
                default:
                    throw new NotSupportedException($"Unknown unit system {units}");
            }
        }
        public static float GravityFactor(this UnitSystem units)
        {
            switch (units)
            {
                case UnitSystem.SI:
                    return 1;
                case UnitSystem.MMKS:
                    return 1000;
                case UnitSystem.IPS:
                    return 386.0885827f;
                case UnitSystem.FPS:
                    return 32.1740486f;
                default:
                    throw new NotSupportedException($"Unknown unit system {units}");
            }
        }
    }



}
