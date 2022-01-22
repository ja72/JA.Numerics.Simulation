using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace JA.Numerics.Simulation.Spatial
{

    public delegate Vector33 AppliedForce(float time, Pose pose, Vector33 velocity);

    public class Link3 : 
        ITree<Link3>,
        IHasUnits<Link3>
    {
        public static readonly AppliedForce ZeroForce = (t, r, v) => Vector33.Zero;

        #region Factory
        internal Link3(World3 world, Pose position, MassProperties mass, JointProperties joint, float initialDisplacement = 0, float initialSpeed = 0)
        {
            World = world ?? throw new ArgumentNullException(nameof(world));
            Units = world.Units;
            Parent = null;
            World.AddObject(this);
            LocalPosition = position;
            MassProperties = mass;
            JointProperties = joint;
            AppliedForce = ZeroForce;
            InitialSpeed = initialSpeed;
            InitialDisplacement = initialDisplacement;
        }
        internal Link3(Link3 parent, Pose localPosition, MassProperties mass, JointProperties joint, float initialDisplacement = 0, float initialSpeed = 0)
        {
            if (parent == null)
            {
                throw new ArgumentNullException(nameof(parent));
            }
            World = parent.World;
            Units = parent.World.Units;
            Parent = parent;
            World.AddObject(this);
            LocalPosition = localPosition;
            MassProperties = mass;
            JointProperties = joint;
            AppliedForce = ZeroForce;
            InitialSpeed = initialSpeed;
            InitialDisplacement = initialDisplacement;
        }

        Link3(UnitSystem units, World3 world, Link3 parent, MassProperties massProperties, JointProperties jointProperties, Pose localPosition, float initialDisplacement, float initialSpeed, AppliedForce appliedForce)
        {
            Units = units;
            World = world;
            LocalPosition = localPosition;
            Parent = parent;
            MassProperties = massProperties;
            JointProperties = jointProperties;
            InitialDisplacement = initialDisplacement;
            InitialSpeed = initialSpeed;
            AppliedForce = appliedForce;
        }

        public Link3 AddChild(Pose localPosition, MassProperties mass, JointProperties type, float initialDisplacement = 0, float initialSpeed = 0)
            => new Link3(this, localPosition, mass.ConvertTo(Units), type, initialDisplacement, initialSpeed);
        public Link3 SlideAlongX(Pose localPosition, MassProperties mass, float initialDisplacement = 0)
            => AddChild(localPosition, mass, JointType.SlideAlongX, initialDisplacement);
        public Link3 SlideAlongY(Pose localPosition, MassProperties mass, float initialDisplacement = 0)
            => AddChild(localPosition, mass, JointType.SlideAlongY, initialDisplacement);
        public Link3 AddRevolute(Pose localPosition, MassProperties mass, float initialAngle = 0)
            => AddChild(localPosition, mass, JointType.RotateAboutZ, initialAngle);
        public Link3 AddCollarAlongX(Pose localPosition, MassProperties mass, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongX, initialDisplacement)
                    .AddChild(Pose.Origin, mass, JointType.RotateAboutZ, initialAngle);
        public Link3 AddCollarAlongY(Pose localPosition, MassProperties mass, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongY, initialDisplacement)
                    .AddChild(Pose.Origin, mass, JointType.RotateAboutZ, initialAngle);
        public Link3 AddFree(Pose localPosition, MassProperties mass, float initialX, float intialY, float initialAngle)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongX, initialX)
                    .AddChild(Pose.Origin, MassProperties.Zero, JointType.SlideAlongY, intialY)
                    .AddChild(Pose.Origin, mass, JointType.RotateAboutZ, initialAngle);

        public void RemoveChild(Link3 child)
        {
            foreach (var item in World.objects.Where(f => f.Parent == child))
            {
                item.Parent = this;
            }
            World.objects.Remove(child);
        }
        #endregion

        #region Properties
        public UnitSystem Units { get; }
        public Pose LocalPosition { get; }
        public World3 World { get; }
        public Link3 Parent { get; internal set; }
        public IReadOnlyList<Link3> Children { get => World.objects.Where(f => f.Parent == this).ToList().AsReadOnly(); }
        public MassProperties MassProperties { get; private set; }
        public JointProperties JointProperties { get; }
        public float InitialDisplacement { get; set; }
        public float InitialSpeed { get; set; }
        public AppliedForce AppliedForce { get; set; }
        
        #endregion

        #region Derived Properties
        public int Index { get => World.objects.IndexOf(this); }
        public int ParentIndex { get => Parent != null ? Parent.Index : -1; }
        public int Level { get => IsRoot ? 0 : Parent.Level + 1; }
        public bool IsRoot { get => Parent == null; }

        public void AddSolid(MassProperties other)
        {
            MassProperties += other;
        }
        public void RemoveSolid(MassProperties other)
        {
            MassProperties -= other;
        }

        public JointState GetInitial()
        {
            float q = InitialDisplacement;
            float qp = InitialSpeed;
            float qpp = 0;
            float tau = 0;
            switch (JointProperties.Motion)
            {
                case Prescribed.Motion:
                    qpp = JointProperties.JointDriver(0, q, qp);
                    break;
                case Prescribed.Load:
                    tau = JointProperties.JointDriver(0, q, qp);
                    break;
                default:
                    throw new NotSupportedException();
            }
            return new JointState(q, qp, qpp, tau);
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

        #endregion

        public override string ToString()
        {
            return $"[{Index}] Frame: Local={LocalPosition}, Parent#={ParentIndex}";
        }

        public Link3 ConvertTo(UnitSystem target)
        {
            UnitSystem u = Units;
            float fl = JointProperties.Type <= JointType.SlideAlongZ
                ? UnitFactors.Length(u, target) : 1;
            float fa = JointProperties.Type <= JointType.SlideAlongZ
                ? UnitFactors.Force(u, target) : UnitFactors.Torque(u, target);
            float f_in = fl, f_out = fa;
            if (JointProperties.Motion == Prescribed.Motion)
            {
                (f_in, f_out) = (f_out, f_in);
            }
            return new Link3(
                target,
                World,
                Parent,
                MassProperties.ConvertTo(target),
                JointProperties,
                LocalPosition.ConvertFromTo(Units, target),
                fl * InitialDisplacement,
                fl * InitialSpeed,
                (t, r, v) => f_out * AppliedForce(t, new Pose(r.Position / fl, r.Orientation), v / fl));
        }
    }
}
