using System;
using System.ComponentModel;
using System.Linq;
using JA.Numerics;

namespace JA.Numerics.Simulation.Spatial.Solvers
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class ChainStates : ICanConvert<ChainStates>
    {
        public ChainStates(JointState[] state)
        {
            State=state;
        }
        public JointState[] State { get; }
        public int Count { get => State.Length; }

        public ref JointState this[int index] => ref State[index];

        public ChainStates ConvertFromTo(UnitSystem units, UnitSystem target)
        {
            if (units == target) return this;
            return new ChainStates(
                State.Select((s) => s.ConvertFromTo(units, target)).ToArray());
        }

        #region Algebra
        public static ChainStates Negate(ChainStates a)
        {
            var states = new JointState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = -a.State[i];
            }
            return new ChainStates(states);
        }

        public static ChainStates Scale(float factor, ChainStates a)
        {
            var states = new JointState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = factor * a.State[i];
            }
            return new ChainStates(states);
        }

        public static ChainStates Add(ChainStates a, ChainStates b)
        {
            if (a.Count != b.Count)
            {
                throw new ArgumentOutOfRangeException(nameof(b));
            }
            var states = new JointState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = a.State[i] + b.State[i];
            }
            return new ChainStates(states);
        }

        public static ChainStates Subtract(ChainStates a, ChainStates b)
        {
            if (a.Count != b.Count)
            {
                throw new ArgumentOutOfRangeException(nameof(b));
            }
            var states = new JointState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = a.State[i] - b.State[i];
            }
            return new ChainStates(states);
        }

        public static ChainStates operator +(ChainStates a, ChainStates b) => Add(a, b);
        public static ChainStates operator -(ChainStates a) => Negate(a);
        public static ChainStates operator -(ChainStates a, ChainStates b) => Subtract(a, b);
        public static ChainStates operator *(float f, ChainStates a) => Scale(f, a);
        public static ChainStates operator *(ChainStates a, float f) => Scale(f, a);
        public static ChainStates operator /(ChainStates a, float d) => Scale(1 / d, a);
        #endregion

        public override string ToString()
        {
            return $"ChainState(Count={State.Length})";
        }

    }
}
