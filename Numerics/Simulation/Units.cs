using System;

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

    public static class UnitFactors
    {
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

}
