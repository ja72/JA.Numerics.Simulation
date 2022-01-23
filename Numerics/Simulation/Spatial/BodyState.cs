using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public readonly struct BodyState : IHasUnits<BodyState>
    {
        public BodyState(UnitSystem units, Pose pose) : this(units, pose, Vector33.Zero) { }
        public BodyState(UnitSystem units, Pose pose, Vector33 momentum) : this()
        {
            Units = units;
            Pose = pose;
            Momentum = momentum;
        }
        [Browsable(false)]
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
}
