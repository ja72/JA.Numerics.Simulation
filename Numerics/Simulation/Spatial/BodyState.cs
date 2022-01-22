using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics.Simulation.Spatial
{
    public readonly struct BodyState :
        IHasUnits<BodyState>
    {
        public BodyState(UnitSystem units, Pose pose) : this(units, pose, Vector33.Zero) { }
        public BodyState(UnitSystem units, Pose pose, Vector33 momentum) : this()
        {
            Units = units;
            Pose = pose;
            Momentum = momentum;
        }
        public UnitSystem Units { get; }
        public Pose Pose { get; }
        public Vector33 Momentum { get; }

        public BodyState ConvertTo(UnitSystem target)
        {
            if (Units == target) return this;
            float fp = UnitFactors.Mass(Units, target)* UnitFactors.Length(Units, target);
            return new BodyState(target,
                Pose.ConvertFromTo(Units, target),
                fp * Momentum);
        }


        #region Algebra
        public static BodyState Negate(BodyState a)
            => new BodyState(a.Units,
                -a.Pose,
                -a.Momentum);
        public static BodyState Scale(float factor, BodyState a)
            => new BodyState(a.Units,
                factor * a.Pose,
                factor * a.Momentum);
        public static BodyState Add(BodyState a, BodyState b)
        {
            b = b.ConvertTo(a.Units);
            return new BodyState(a.Units,
                a.Pose + b.Pose,
                a.Momentum + b.Momentum);
        }

        public static BodyState Subtract(BodyState a, BodyState b)
        {
            b = b.ConvertTo(a.Units);
            return new BodyState(a.Units,
                a.Pose - b.Pose,
                a.Momentum - b.Momentum);
        }

        public static BodyState operator +(BodyState a, BodyState b) => Add(a, b);
        public static BodyState operator -(BodyState a) => Negate(a);
        public static BodyState operator -(BodyState a, BodyState b) => Subtract(a, b);
        public static BodyState operator *(float f, BodyState a) => Scale(f, a);
        public static BodyState operator *(BodyState a, float f) => Scale(f, a);
        public static BodyState operator /(BodyState a, float d) => Scale(1 / d, a);
        #endregion

    }

    public class WorldState : IHasUnits<WorldState>
    {
        public WorldState(UnitSystem units, params BodyState[] state)
        {
            Units = units;
            State = state;
        }

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
