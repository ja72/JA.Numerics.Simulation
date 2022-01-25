using System;
using System.ComponentModel;
using JA.Numerics.UI;
using System.Drawing;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Geometry : IHasUnits<Geometry>, IVisible
    {
        #region Factory
        protected Geometry(Mesh mesh, World3 world)
            : this(mesh, world, Pose.Origin) { }
        protected Geometry(Mesh mesh, World3 world, Pose meshOrigin)
        {
            World = world ?? throw new ArgumentNullException(nameof(world));
            MeshOrigin=meshOrigin;
            Mesh=mesh;
        }
        protected Geometry(Geometry copy)
        {
            World = copy.World;
            MeshOrigin=copy.MeshOrigin;
            Mesh=copy.Mesh;
        }
        protected Geometry(Geometry copy, UnitSystem target)
        {
            World = copy.World;
            MeshOrigin=copy.MeshOrigin.ConvertFromTo(copy.Units, target);
            Mesh=copy.Mesh.ConvertTo(target);
        }

        #endregion

        #region Properties
        [Browsable(false)]
        public UnitSystem Units { get => Mesh.Units; }
        [Browsable(false)]
        public World3 World { get; }
        public Mesh Mesh { get; set; }
        public Pose MeshOrigin { get; set; }
        #endregion

        public Geometry ConvertTo(UnitSystem target)
        {
            return new Geometry(this, target);
        }

        public void Render(Graphics g, Camera camera)
        {
            Mesh.Render(g, camera, MeshOrigin, true);
        }

        public override string ToString()
        {
            return $"Geometry({Mesh}, Pos={MeshOrigin.Position})";
        }

    }
}
