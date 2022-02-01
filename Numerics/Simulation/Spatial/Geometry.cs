using System;
using System.ComponentModel;
using JA.UI;
using System.Drawing;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Geometry : Object, 
        IHasUnits<Geometry>
    {
        #region Factory
        protected Geometry(Mesh mesh, World3 world)
            : this(mesh, world, Pose.Origin) { }
        protected Geometry(Mesh mesh, World3 world, Pose meshOrigin)
            : base(world)
        {
            MeshOrigin=meshOrigin;
            Mesh=mesh.ConvertTo(world.Units);
        }
        protected Geometry(Geometry copy)
            : base( copy )
        {
            MeshOrigin=copy.MeshOrigin;
            Mesh=copy.Mesh;
        }
        protected Geometry(Geometry copy, UnitSystem target)
            : base(copy, target)
        {
            MeshOrigin=copy.MeshOrigin.ConvertFromTo(copy.Units, target);
            Mesh=copy.Mesh.ConvertTo(target);
        }

        #endregion
        #region Properties
        public Mesh Mesh { get; set; }
        public Pose MeshOrigin { get; set; }

        #endregion

        #region Methods
        public Geometry ConvertTo(UnitSystem target)
        {
            return new Geometry(this, target);
        }
        protected override Object InnerConvertTo(UnitSystem target)
        {
            return ConvertTo(target);
        }

        public override void Render(Graphics g, Camera camera, Pose pose)
        {
            pose = Pose.FromLocal(pose, MeshOrigin);
            Mesh?.Render(g, camera, pose, false);
        }

        public override string ToString()
        {
            return $"Geometry({Mesh}, Pos={MeshOrigin.Position})";
        } 
        #endregion

    }
}
