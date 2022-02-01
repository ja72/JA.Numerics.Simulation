using System.ComponentModel;
using System.Drawing;
using System.Numerics;
using JA.UI;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Body : Geometry
    {
        public Body(Mesh mesh, World3 world, MassProperties massProperties, Pose localPosition)
            : this(mesh, world.Units, world, massProperties, localPosition) { }
        public Body(Mesh mesh, UnitSystem units, World3 world, MassProperties massProperties, Pose localPosition)
            : base(mesh.ConvertTo(units), world, localPosition.ConvertFromTo(mesh.Units, units))
        {
            MassProperties=massProperties;
            AppliedForce = World3.ZeroForce;
        }
        protected Body(Body copy) : base(copy)
        {
            MassProperties=copy.MassProperties;
            AppliedForce = copy.AppliedForce;
        }
        protected Body(Body copy, UnitSystem target)
            : base(copy, target)
        {
            MassProperties=copy.MassProperties.ConvertTo(target);

            var u = copy.Units;

            if (u!=target && copy.AppliedForce != null && copy.AppliedForce != World3.ZeroForce)
            {
                AppliedForce = (t, r, v) => copy.AppliedForce(t,
                    r.ConvertFromTo(target, u),
                    v.ConvertFromTo(target, u, UnitType.Length, ScrewType.Twist)
                ).ConvertFromTo(u, target, UnitType.Force, ScrewType.Wrench);
            }
            else
            {
                AppliedForce = copy.AppliedForce;
            }
        }

        public AppliedForce AppliedForce { get; set; }
        public MassProperties MassProperties { get; private set; }
        public Vector3 CG { get => Pose.FromLocal(MeshOrigin, MassProperties.CG); }
        public void AddSolid(MassProperties other)
        {
            MassProperties += other;
        }
        public void RemoveSolid(MassProperties other)
        {
            MassProperties -= other;
        }
        public override string ToString()
        {
            return $"Body(Mass={MassProperties.Mass}, CG={CG})";
        }

    }
}
