using System.Collections.Generic;
using System.ComponentModel;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public readonly struct BodyState : ICanConvert<BodyState>
    {
        public BodyState(Pose pose) : this(pose, Vector33.Zero) { }
        public BodyState(Pose pose, Vector33 momentum) : this()
        {
            Pose = pose;
            Momentum = momentum;
        }
        public Pose Pose { get; }
        public Vector33 Momentum { get; }

        public BodyState ConvertFromTo(UnitSystem units, UnitSystem target)
        {
            if (units == target) return this;
            float fl = UnitFactors.Length(units, target);
            float fm = UnitFactors.Momentum(units, target);
            float fmm = UnitFactors.AngularMomentum(units, target);
            return new BodyState(
                new Pose(fl*Pose.Position, Pose.Orientation),
                new Vector33(fm*Momentum.Vector1, fmm*Momentum.Vector2));
        }


        #region Algebra
        public static BodyState Negate(BodyState a)
            => new BodyState(
                -a.Pose,
                -a.Momentum);
        public static BodyState Scale(float factor, BodyState a)
            => new BodyState(
                factor * a.Pose,
                factor * a.Momentum);
        public static BodyState Add(BodyState a, BodyState b) 
            => new BodyState(
                a.Pose + b.Pose,
                a.Momentum + b.Momentum);

        public static BodyState Subtract(BodyState a, BodyState b) 
            => new BodyState(
                a.Pose - b.Pose,
                a.Momentum - b.Momentum);

        public static BodyState operator +(BodyState a, BodyState b) => Add(a, b);
        public static BodyState operator -(BodyState a) => Negate(a);
        public static BodyState operator -(BodyState a, BodyState b) => Subtract(a, b);
        public static BodyState operator *(float f, BodyState a) => Scale(f, a);
        public static BodyState operator *(BodyState a, float f) => Scale(f, a);
        public static BodyState operator /(BodyState a, float d) => Scale(1 / d, a);
        #endregion

        public override string ToString()
        {
            return $"State(Pos={Pose.Position}, Mom={Momentum.Vector1})";
        }

    }
}
