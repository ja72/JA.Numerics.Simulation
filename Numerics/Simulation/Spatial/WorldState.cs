using System;
using System.ComponentModel;
using System.Linq;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class WorldState : IHasUnits<WorldState>
    {
        public WorldState(UnitSystem units, params BodyState[] state)
        {
            Units = units;
            State = state;
        }
        [Browsable(false)]
        public UnitSystem Units { get; }
        public BodyState[] State { get; }
        public int Count { get => State.Length; }

        public WorldState ConvertTo(UnitSystem target)
        {
            if (Units == target) return this;
            return new WorldState(target,
                State.Select((s) => s.ConvertTo(target)).ToArray());
        }

        #region Algebra
        public static WorldState Negate(WorldState a)
        {
            var states = new BodyState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = -a.State[i];
            }
            return new WorldState(a.Units, states);
        }

        public static WorldState Scale(float factor, WorldState a)
        {
            var states = new BodyState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = factor * a.State[i];
            }
            return new WorldState(a.Units, states);
        }

        public static WorldState Add(WorldState a, WorldState b)
        {
            if (a.Count != b.Count)
            {
                throw new ArgumentOutOfRangeException(nameof(b));
            }
            var states = new BodyState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = a.State[i] + b.State[i];
            }
            return new WorldState(a.Units, states);
        }

        public static WorldState Subtract(WorldState a, WorldState b)
        {
            if (a.Count != b.Count)
            {
                throw new ArgumentOutOfRangeException(nameof(b));
            }
            var states = new BodyState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = a.State[i] - b.State[i];
            }
            return new WorldState(a.Units, states);
        }

        public static WorldState operator +(WorldState a, WorldState b) => Add(a, b);
        public static WorldState operator -(WorldState a) => Negate(a);
        public static WorldState operator -(WorldState a, WorldState b) => Subtract(a, b);
        public static WorldState operator *(float f, WorldState a) => Scale(f, a);
        public static WorldState operator *(WorldState a, float f) => Scale(f, a);
        public static WorldState operator /(WorldState a, float d) => Scale(1 / d, a);
        #endregion

    }
}
