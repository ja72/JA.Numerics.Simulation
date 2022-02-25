using System.ComponentModel;
using System.Drawing;
using System.Numerics;
using JA.UI;

namespace JA.Numerics.Simulation.Spatial
{
    using Geometry;

    /// <summary>
    /// Simulation Body with <see cref="Spatial.MassProperties"/> on top of <see cref="Shape" />
    /// </summary>
    /// <seealso cref="Shape" />
    /// <seealso cref="Solvers.Link3" />
    /// <seealso cref="Spatial.MassProperties"/>
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Body : Shape
    {
        private MassProperties massProperties;

        public Body(Mesh mesh, World3 world, MassProperties massProperties, Pose localPosition)
            : this(mesh, world.Units, world, massProperties, localPosition) { }
        public Body(Mesh mesh, UnitSystem units, World3 world, MassProperties massProperties, Pose localPosition)
            : base(mesh.ConvertTo(units), world, localPosition.ConvertFromTo(mesh.Units, units))
        {
            MassProperties=massProperties.ConvertTo(units);
            AppliedForce = World3.ZeroForce;
        }
        protected Body(Body copy) : base(copy)
        {
            MassProperties=copy.MassProperties;
            AppliedForce = copy.AppliedForce;
        }

        public AppliedForce AppliedForce { get; set; }
        public MassProperties MassProperties
        {
            get => massProperties;
            set
            {
                massProperties=value.ConvertTo(Units);
            }
        }
        public Vector3 CG { get => Pose.FromLocal(MeshOrigin, massProperties.CG); }
        public void AddSolid(MassProperties other)
        {
            massProperties += other;
        }
        public void RemoveSolid(MassProperties other)
        {
            massProperties -= other;
        }
        public override string ToString()
        {
            return $"Body(Mass={MassProperties.Mass}, CG={CG})";
        }
        public override void ConvertTo(UnitSystem target)
        {
            massProperties = massProperties.ConvertTo(target);

            var force = AppliedForce;

            if (force != null && force != World3.ZeroForce)
            {
                var u = Units;
                float ft = Unit.Time.Convert(Units, target);
                AppliedForce = (t, r, v) => force(
                    t/ft,
                    r.ConvertFromTo(target, u),
                    v.ConvertFromTo(target, u, Unit.RotationalSpeed, ScrewType.Twist))
                .ConvertFromTo(u, target, Unit.Force, ScrewType.Wrench);
            }
            base.ConvertTo(target);
        }
    }
}
