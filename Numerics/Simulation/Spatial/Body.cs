using System.ComponentModel;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Body : Geometry
    {
        protected Body(World3 world, MassProperties massProperties, Pose localPosition)
            : this(null, world, massProperties, localPosition) { }
        protected Body(Mesh mesh, World3 world, MassProperties massProperties, Pose localPosition)
            : this(mesh, world.Units, world, massProperties, localPosition) { }
        protected Body(Mesh mesh, UnitSystem units, World3 world, MassProperties massProperties, Pose localPosition)
            : base(mesh, world, localPosition)
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

            float fl = UnitFactors.Length(copy.Units, target);
            float ff = UnitFactors.Force(copy.Units, target);

            if (copy.AppliedForce != null && copy.AppliedForce != World3.ZeroForce)
            {
                AppliedForce = (t, r, v) => ff * copy.AppliedForce(t, r.ConvertFromTo(target, u), new Vector33(v.Vector1/fl, v.Vector2));
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
