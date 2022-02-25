using System;
using System.ComponentModel;
using JA.UI;
using System.Drawing;

namespace JA.Numerics.Simulation.Spatial
{
    using Geometry;

    /// <summary>
    /// Simulation Geometry containing a <see cref="Spatial.Mesh"/> on top of <see cref="Object"/>
    /// </summary>
    /// <seealso cref="Object" />
    /// <seealso cref="Body" />
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Shape : Object
    {
        #region Factory
        protected Shape(Mesh mesh, World3 world)
            : this(mesh, world, Pose.Origin) { }
        protected Shape(Mesh mesh, World3 world, Pose meshOrigin)
            : base(world)
        {
            MeshOrigin=meshOrigin;
            Mesh=new Mesh(mesh);
            Mesh.ConvertTo(world.Units);
        }
        protected Shape(Shape copy)
            : base( copy )
        {
            MeshOrigin=copy.MeshOrigin;
            Mesh=new Mesh(copy.Mesh);
        }

        #endregion
        #region Properties
        public Mesh Mesh { get; set; }
        public Pose MeshOrigin { get; set; }

        #endregion

        #region Methods
        public override void ConvertTo(UnitSystem target)
        {
            MeshOrigin = MeshOrigin.ConvertFromTo(Units, target);
            Mesh = Mesh.ConvertTo(target);

            base.ConvertTo(target);
        }

        public override void Render(Graphics g, Camera camera, Pose pose, float scale = 1)
        {
            pose = Pose.FromLocal(pose, MeshOrigin);
            Mesh?.Render(g, camera, pose, false, scale);
        }

        public override string ToString()
        {
            return $"Geometry({Mesh}, Pos={MeshOrigin.Position})";
        } 
        #endregion

    }
}
