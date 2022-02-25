using System;
using System.ComponentModel;
using System.Linq;
using JA.Numerics;

namespace JA.Numerics.Simulation.Spatial.Solvers
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class MbdStates 
    {
        public MbdStates(params BodyState[] state)
        {
            State = state;
        }
        public BodyState[] State { get; }
        public int Count { get => State.Length; }

        public ref BodyState this[int index] => ref State[index];

        #region Algebra
        public static MbdStates Negate(MbdStates a)
        {
            var states = new BodyState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = -a.State[i];
            }
            return new MbdStates(states);
        }

        public static MbdStates Scale(float factor, MbdStates a)
        {
            var states = new BodyState[a.Count];
            for (int i = 0; i < states.Length; i++)
            {
                states[i] = factor * a.State[i];
            }
            return new MbdStates(states);
        }

        public static MbdStates Add(MbdStates a, MbdStates b)
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
            return new MbdStates(states);
        }

        public static MbdStates Subtract(MbdStates a, MbdStates b)
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
            return new MbdStates(states);
        }

        public static MbdStates operator +(MbdStates a, MbdStates b) => Add(a, b);
        public static MbdStates operator -(MbdStates a) => Negate(a);
        public static MbdStates operator -(MbdStates a, MbdStates b) => Subtract(a, b);
        public static MbdStates operator *(float f, MbdStates a) => Scale(f, a);
        public static MbdStates operator *(MbdStates a, float f) => Scale(f, a);
        public static MbdStates operator /(MbdStates a, float d) => Scale(1 / d, a);
        #endregion

        public override string ToString()
        {
            return $"MbdState(Count={Count})";
        }
    }
}
