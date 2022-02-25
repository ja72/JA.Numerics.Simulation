using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using JA.UI;

namespace JA.Numerics.Simulation.Spatial.Solvers
{
    using Geometry;

    /// <summary>
    /// Link3 simulation object containing <see cref="Solvers.JointProperties"/> on top of <see cref="Body"/>
    /// </summary>
    /// <seealso cref="Body" />
    /// <seealso cref="Solvers.JointProperties"/>
    /// <seealso cref="Chain"/>
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Link3 : Body
    {
        #region Factory
        internal Link3(Mesh mesh, Chain chain, Pose meshPosition, MassProperties mass, Pose onWorld, JointProperties joint, float initialDisplacement = 0)
            : base(mesh, chain.World, mass, meshPosition)
        {
            OnChain = chain;
            Parent = null;
            OnChain.AddLink(this);
            JointProperties = joint;
            LocationOnParent = onWorld;
            InitialDisplacement = initialDisplacement;
            InitialSpeed = 0;
            LocalMarker = 2*CG;
        }
        internal Link3(Mesh mesh, Link3 parent, Pose meshPosition, MassProperties mass, Pose onParent, JointProperties joint, float initialDisplacement = 0)
            : base(mesh, parent.World, mass, meshPosition)
        {
            OnChain = parent.OnChain;
            Parent = parent??throw new ArgumentNullException(nameof(parent));
            OnChain.AddLink(this);
            JointProperties = joint;
            LocationOnParent = onParent;
            InitialDisplacement = initialDisplacement;
            InitialSpeed = 0;
            LocalMarker = 2*CG;
        }
        internal Link3(Body body, Link3 parent, Pose onParent, JointProperties joint, float initialDisplacement = 0)
            : base(body.Mesh, body.World, body.MassProperties, body.MeshOrigin)
        {
            OnChain = parent.OnChain;
            Parent = parent??throw new ArgumentNullException(nameof(parent));
            OnChain.AddLink(this);
            JointProperties = joint;
            LocationOnParent = onParent;
            InitialDisplacement = initialDisplacement;
            InitialSpeed = 0f;
            LocalMarker = 2*CG;
        }
        internal Link3(Body body, Chain chain, Pose onParent, JointProperties joint, float initialDisplacement = 0)
            : base(body.Mesh, chain.World, body.MassProperties, body.MeshOrigin)
        {
            OnChain = chain;
            Parent = null;
            OnChain.AddLink(this);
            JointProperties = joint;
            LocationOnParent = onParent;
            InitialDisplacement = initialDisplacement;
            InitialSpeed = 0f;
            LocalMarker = 2*CG;
        }
        public Link3(Link3 copy) : base(copy)
        {
            OnChain =copy.OnChain;
            Parent = copy.Parent;
            AppliedForce = copy.AppliedForce;
            JointProperties = copy.JointProperties;
            InitialDisplacement = copy.InitialDisplacement;
            InitialSpeed = copy.InitialSpeed;
            LocationOnParent = copy.LocationOnParent;
            LocalMarker = copy.LocalMarker;
        }


        public Link3 AddChild(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onParent, JointProperties type, float initialDisplacement = 0)
            => new Link3(mesh, this, meshPosition, mass.ConvertTo(Units), onParent, type, initialDisplacement);
        public Link3 AddChild(Body body, Pose onParent, JointProperties type, float initialDisplacement = 0)
            => new Link3(body, this, onParent, type, initialDisplacement);

        public Link3 SlideAlongX(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onParent, float initialDisplacement = 0)
            => AddChild(mesh, meshPosition, mass, onParent, JointType.SlideAlongX, initialDisplacement);

        public Link3 SlideAlongY(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onParent, float initialDisplacement = 0)
            => AddChild(mesh, meshPosition, mass, onParent, JointType.SlideAlongY, initialDisplacement);
        public Link3 AddRevolute(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onParent, float initialAngle = 0)
            => AddChild(mesh, meshPosition, mass, onParent, JointType.RotateAboutZ, initialAngle);
        public Link3 AddCollarAlongX(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onParent, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(null, meshPosition, MassProperties.Zero, onParent, JointType.SlideAlongX, initialDisplacement)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);

        public Link3 AddCollarAlongY(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onParent, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(null, meshPosition, MassProperties.Zero, onParent, JointType.SlideAlongY, initialDisplacement)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);
        public Link3 AddFree(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onParent, float initialX, float intialY, float initialAngle)
            => AddChild(null, meshPosition, MassProperties.Zero, onParent, JointType.SlideAlongX, initialX)
                    .AddChild(null, Pose.Origin, MassProperties.Zero, Pose.Origin, JointType.SlideAlongY, intialY)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);

        public void RemoveChild(Link3 child)
        {
            OnChain.RemoveChild(child);
        }
        #endregion

        #region Tree
        [Category("Model")] public Link3 Parent { get; private set; }
        [Category("Model")] public int Level { get => IsRoot ? 0 : Parent.Level + 1; }
        [Category("Model")] public bool IsRoot { get => Parent == null; }
        public void SetParent(Link3 parent)
        {
            this.Parent = parent;
        }
        public void MakeRoot()
        {
            this.Parent = null;
        }

        #endregion

        #region Properties        
        public Chain OnChain { get; }
        public Pose LocationOnParent { get; set; }
        public JointProperties JointProperties { get; }
        public float InitialDisplacement { get; set; }
        public float InitialSpeed { get; set; }
        public Vector3 LocalMarker { get; set; }

        #endregion

        #region Derived Properties

        public JointState GetInitial()
        {
            return new JointState(JointProperties.Type, InitialDisplacement, InitialSpeed);
        }
        #endregion

        #region Methods

        public override string ToString()
        {
            return $"Link3(Local={MeshOrigin}, {JointProperties}, {MassProperties})";
        }

        public override void ConvertTo(UnitSystem target)
        {
            if (Units == target) return;
            LocationOnParent *= Unit.Length.Convert(Units, target);
            InitialDisplacement *= Unit.Length.Convert(Units, target);
            InitialSpeed *= Unit.Speed.Convert(Units, target);
            LocalMarker *= Unit.Length.Convert(Units, target);
            JointProperties.ConvertFromTo(Units, target);
            base.ConvertTo(target);
        }

        public override void Render(Graphics g, Camera camera, Pose pose, float scale = 1)
        {
            base.Render(g, camera, pose);
            
            var dir = JointProperties.GetDirection(pose);
            var pos = pose.Position;

            // var scale = camera.ProjectionScale*camera.EyeDistance/2;
            scale *= 1.2f;
            var pts = camera.Project(pos-scale*dir/2, pos, pos+scale*dir/2);
            using var pen = new Pen(Color.FromArgb(128, Color.Black), 0);
            pen.CustomEndCap = new AdjustableArrowCap(2f, 8f, true);
            pen.DashStyle = DashStyle.DashDot;
            g.DrawLines(pen, pts);

            var r_marker = Pose.FromLocal(pose, LocalMarker);
            pts = camera.Project(r_marker);
            g.FillEllipse(Brushes.HotPink, pts[0].X-2, pts[0].Y-2, 4, 4);

            var r_cg = Pose.FromLocal(pose, CG);
            pts = camera.Project(r_cg);
            g.FillEllipse(Brushes.Black, pts[0].X-2, pts[0].Y-2, 4, 4);
        }

        #endregion
    }
}
