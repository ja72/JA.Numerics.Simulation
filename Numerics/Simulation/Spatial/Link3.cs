using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using JA.Numerics.UI;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Link3 : Body, 
        IHasUnits<Link3>
    {

        #region Factory
        internal Link3(Mesh mesh, World3 world, Pose meshPosition, MassProperties mass, Pose onWorld, JointProperties joint, float initialDisplacement = 0)
            : base(mesh, world, mass, meshPosition)
        {
            Parent = null;
            World.AddLink(this);
            JointProperties = joint;
            LocationOnParent = onWorld;
            InitialDisplacement = initialDisplacement;
            InitialSpeed = 0;
        }
        internal Link3(Mesh mesh, Link3 parent, Pose meshPosition, MassProperties mass, Pose onParent, JointProperties joint, float initialDisplacement = 0)
            : base(mesh, parent.World, mass, meshPosition)
        {
            Parent = parent??throw new ArgumentNullException(nameof(parent));
            World.AddLink(this);
            JointProperties = joint;
            LocationOnParent = onParent;
            InitialDisplacement = initialDisplacement;
            InitialSpeed = 0;
        }

        public Link3(Link3 copy) : base(copy)
        {
            this.Parent = copy.Parent;
            this.AppliedForce = copy.AppliedForce;
            this.JointProperties = copy.JointProperties;
            this.InitialDisplacement = copy.InitialDisplacement;
            this.InitialSpeed = copy.InitialSpeed;
            this.LocationOnParent = copy.LocationOnParent;
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
            this.Parent = copy.Parent;
            this.LocationOnParent = copy.LocationOnParent.ConvertFromTo(u, target);
            this.JointProperties = copy.JointProperties.ConvertFromTo(u, target);
            this.InitialDisplacement = fs*copy.InitialDisplacement;
            this.InitialSpeed = fs*copy.InitialSpeed;
            this.AppliedForce = (t, r, v) => f_out * copy.AppliedForce(t, new Pose(r.Position / fs, r.Orientation), v / fs);
        }

        public Link3 AddChild(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onParent, JointProperties type, float initialDisplacement = 0)
            => new Link3(mesh, this, meshPosition, mass.ConvertTo(Units), onParent, type, initialDisplacement);

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
            => AddChild(null,meshPosition, MassProperties.Zero, onParent, JointType.SlideAlongX, initialX)
                    .AddChild(null, Pose.Origin, MassProperties.Zero, Pose.Origin, JointType.SlideAlongY, intialY)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);

        public void RemoveChild(Link3 child)
        {
            World.RemoveChild(child);
        }
        #endregion

        #region Tree
        [Category("Model")] public Link3 Parent { get; private set; }
        [Category("Model")] public IList<Link3> Children { get => World.linkList.Where(f => f.Parent == this).ToList(); }
        [Category("Model")] public int Index { get => World.linkList.IndexOf(this); }
        [Category("Model")] public int ParentIndex { get => Parent != null ? Parent.Index : -1; }
        [Category("Model")] public int Level { get => IsRoot ? 0 : Parent.Level + 1; }
        [Category("Model")] public bool IsRoot { get => Parent == null; }
        public void SetParent(Link3 parent) => Parent = parent;

        #endregion

        #region Properties
        public Pose LocationOnParent { get; set; }
        public JointProperties JointProperties { get; }
        public float InitialDisplacement { get; set; }
        public float InitialSpeed { get; set; }

        #endregion

        #region Derived Properties

        public JointState GetInitial()
        {
            return new JointState(JointProperties.Type, InitialDisplacement, InitialSpeed);
        }
        #endregion

        #region Methods
        /// <summary>
        /// Gets the pose across the joint given the joint angle q.
        /// </summary>
        /// <param name="q">The joint angle/distance.</param>
        public Pose GetLocalStep(float q)
        {
            switch (JointProperties.Type)
            {
                case JointType.SlideAlongX:
                    return Pose.AlongX(q);
                case JointType.SlideAlongY:
                    return Pose.AlongY(q);
                case JointType.SlideAlongZ:
                    return Pose.AlongZ(q);
                case JointType.RotateAboutX:
                    return Pose.AboutX(q);
                case JointType.RotateAboutY:
                    return Pose.AboutY(q);
                case JointType.RotateAboutZ:
                    return Pose.AboutZ(q);
                default:
                    throw new NotSupportedException("Unknown joint type.");
            }
        }
        /// <summary>
        /// Gets the joint axis.
        /// </summary>
        /// <param name="pose">The joint pose.</param>
        public Vector33 GetAxis(Pose pose)
        {
            switch (JointProperties.Type)
            {
                case JointType.SlideAlongX:
                    return Vector33.Twist(Vector3.UnitX.Rotate(pose.Orientation));
                case JointType.SlideAlongY:
                    return Vector33.Twist(Vector3.UnitY.Rotate(pose.Orientation));
                case JointType.SlideAlongZ:
                    return Vector33.Twist(Vector3.UnitZ.Rotate(pose.Orientation));
                case JointType.RotateAboutX:
                    return Vector33.Twist(Vector3.UnitX.Rotate(pose.Orientation), pose.Position, 0);
                case JointType.RotateAboutY:
                    return Vector33.Twist(Vector3.UnitY.Rotate(pose.Orientation), pose.Position, 0);
                case JointType.RotateAboutZ:
                    return Vector33.Twist(Vector3.UnitZ.Rotate(pose.Orientation), pose.Position, 0);
                default:
                    throw new NotSupportedException("Unknown joint type.");
            }
        }


        public override string ToString()
        {
            return $"Link3(Local={MeshOrigin}, Index={Index}, Parent#={ParentIndex})";
        }

        public new Link3 ConvertTo(UnitSystem target)
        {
            return new Link3(this, target);
        }

        #endregion
    }
}
