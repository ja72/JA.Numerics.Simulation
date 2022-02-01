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
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Link3 : Body,
        IHasUnits<Link3>
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
        public Link3(Link3 copy, UnitSystem target) : base(copy, target)
        {
            UnitSystem u = copy.Units;
            float fl = copy.JointProperties.Type <= JointType.SlideAlongZ
                ? UnitFactors.Length(u, target) : 1;
            float fa = copy.JointProperties.Type <= JointType.SlideAlongZ
                ? UnitFactors.Force(u, target) : UnitFactors.Torque(u, target);
            float f_in = fl, f_out = fa;
            if (copy.JointProperties.Motion == Prescribed.Motion)
            {
                (f_in, f_out) = (f_out, f_in);
            }

            float fs = UnitFactors.Length(copy.Units, target);
            OnChain = copy.OnChain;
            Parent = copy.Parent;
            LocationOnParent = copy.LocationOnParent.ConvertFromTo(u, target);
            JointProperties = copy.JointProperties.ConvertFromTo(u, target);
            InitialDisplacement = fs*copy.InitialDisplacement;
            InitialSpeed = fs*copy.InitialSpeed;
            LocalMarker = fl*copy.LocalMarker;
            AppliedForce = (t, r, v) => copy.AppliedForce(
                t, 
                r.ConvertFromTo(target, u), 
                v.ConvertFromTo(target, u, UnitType.Length, ScrewType.Twist)
            ).ConvertFromTo(u, target, UnitType.Force, ScrewType.Wrench);
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
        [Category("Model")] public IList<Link3> Children { get => OnChain.linkList.Where(f => f.Parent == this).ToList(); }
        [Category("Model")] public int Index { get => OnChain.linkList.IndexOf(this); }
        [Category("Model")] public int ParentIndex { get => Parent != null ? Parent.Index : -1; }
        [Category("Model")] public int Level { get => IsRoot ? 0 : Parent.Level + 1; }
        [Category("Model")] public bool IsRoot { get => Parent == null; }
        public void SetParent(Link3 parent) => Parent = parent;

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
            return $"Link3(Local={MeshOrigin}, Index={Index}, Parent#={ParentIndex})";
        }

        public new Link3 ConvertTo(UnitSystem target)
        {
            return new Link3(this, target);
        }

        public override void Render(Graphics g, Camera camera, Pose pose)
        {
            base.Render(g, camera, pose);
            
            var dir = JointProperties.GetDirection(pose);
            var pos = pose.Position;

            var pts = camera.Project(pos-dir, pos, pos+dir);
            float scale = 1;
            using var pen = new Pen(Color.FromArgb(128, Color.Black), 0);
            pen.CustomEndCap = new AdjustableArrowCap(2f*scale, 8f*scale, true);
            pen.DashStyle = DashStyle.DashDot;
            g.DrawLines(pen, pts);

            var r_marker = Pose.FromLocal(pose, LocalMarker);
            pts = camera.Project(r_marker);
            g.FillEllipse(Brushes.HotPink, pts[0].X-2, pts[0].Y-2, 4, 4);

            var r_cg = Pose.FromLocal(pose, MassProperties.CG);
            pts = camera.Project(r_cg);
            g.FillEllipse(Brushes.Black, pts[0].X-2, pts[0].Y-2, 4, 4);
        }

        #endregion
    }
}
