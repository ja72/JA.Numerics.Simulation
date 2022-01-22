using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace JA.Numerics.Simulation.Planar
{

    public delegate Vector21 AppliedForce(float time, Pose pose, Vector21 velocity);

    public class Frame2
    {
        public static readonly JointDriver ZeroDriver = (t, q, qp) => 0;
        public static readonly AppliedForce ZeroForce = (t, r, v) => Vector21.Zero;

        #region Factory
        internal Frame2(World2 world, Pose position, MassProperties mass, JointProperties joint, float initialDisplacement = 0, float initialSpeed = 0)
        {
            this.World = world;
            this.Parent = null;
            this.World.AddObject(this);
            this.LocalPosition = position;
            this.MassProperties = mass;
            this.JointProperties = joint;
            this.AppliedForce = ZeroForce;
            this.JointDriver = ZeroDriver;
            this.InitialSpeed = initialSpeed;
            this.InitialDisplacement = initialDisplacement;
        }
        internal Frame2(Frame2 parent, Pose localPosition, MassProperties mass, JointProperties joint, float initialDisplacement = 0, float initialSpeed = 0)
        {
            this.World = parent.World;
            this.Parent = parent;
            this.World.AddObject(this);
            this.LocalPosition = localPosition;
            this.MassProperties = mass;
            this.JointProperties = joint;
            this.AppliedForce = ZeroForce;
            this.JointDriver = ZeroDriver;
            this.InitialSpeed = initialSpeed;
            this.InitialDisplacement = initialDisplacement;
        }

        public Frame2 AddChild(Pose localPosition, MassProperties mass, JointProperties type, float initialDisplacement = 0, float initialSpeed = 0)
            => new Frame2(this, localPosition, mass, type, initialDisplacement, initialSpeed);
        public Frame2 SlideAlongX(Pose localPosition, MassProperties mass, float initialDisplacement = 0)
            => AddChild(localPosition, mass, JointType.SlideAlongX, initialDisplacement);
        public Frame2 SlideAlongY(Pose localPosition, MassProperties mass, float initialDisplacement = 0)
            => AddChild(localPosition, mass, JointType.SlideAlongY, initialDisplacement);
        public Frame2 AddRevolute(Pose localPosition, MassProperties mass, float initialAngle = 0)
            => AddChild(localPosition, mass, JointType.Revolute, initialAngle);
        public Frame2 AddCollarAlongX(Pose localPosition, MassProperties mass, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongX, initialDisplacement)
                    .AddChild(Pose.Origin, mass, JointType.Revolute, initialAngle);
        public Frame2 AddCollarAlongY(Pose localPosition, MassProperties mass, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongY, initialDisplacement)
                    .AddChild(Pose.Origin, mass, JointType.Revolute, initialAngle);
        public Frame2 AddFree(Pose localPosition, MassProperties mass, float initialX, float intialY, float initialAngle)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongX, initialX)
                    .AddChild(Pose.Origin, MassProperties.Zero, JointType.SlideAlongY, intialY)
                    .AddChild(Pose.Origin, mass, JointType.Revolute, initialAngle);

        public void RemoveChild(Frame2 child)
        {
            foreach (var item in World.objects.Where(f=> f.Parent == child))
            {
                item.Parent = this;
            }
            World.objects.Remove(child);
        }
        #endregion

        #region Properties
        public Pose LocalPosition { get; }
        public World2 World { get; }
        public Frame2 Parent { get; internal set; }
        public IEnumerable<Frame2> Children { get => World.objects.Where(f => f.Parent == this); }
        public MassProperties MassProperties { get; private set; }
        public JointProperties JointProperties { get; }
        public float Mass
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MassProperties.Mass;
        }
        public float MMoi
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => MassProperties.MMoi;
        }
        public JointType Type
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => JointProperties.Type;
        }
        public Prescribed Motion
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => JointProperties.Motion;
        }
        public float InitialDisplacement { get; set; }
        public float InitialSpeed { get; set; }
        public AppliedForce AppliedForce { get; set; }
        public JointDriver JointDriver { get; set; }
        #endregion

        #region Derived Properties
        public int Index { get => World.objects.IndexOf(this); }
        public int ParentIndex { get => Parent != null ? Parent.Index : -1; }
        public int Level { get => IsRoot ? 0 : Parent.Level + 1; }
        public bool IsRoot { get => Parent == null; }

        public void AddSolid(MassProperties other)
        {
            this.MassProperties += other;
        }
        public void RemoveSolid(MassProperties other)
        {
            this.MassProperties -= other;
        }

        public FrameState GetInitial()
        {
            float q = InitialDisplacement;
            float qp = InitialSpeed;
            float qpp = 0;
            float tau = 0;
            switch (JointProperties.Motion)
            {
                case Prescribed.Motion:
                    qpp = JointDriver(0, q, qp);
                    break;
                case Prescribed.Load:
                    tau = JointDriver(0, q, qp);
                    break;
                default:
                    throw new NotSupportedException();
            }
            return new FrameState(q, qp, qpp, tau);
        }
        #endregion

        #region Methods
        /// <summary>
        /// Gets the pose across the joint given the joint angle q.
        /// </summary>
        /// <param name="q">The joint angle/distance.</param>
        public Pose GetLocalStep(float q)
        {
            switch (Type)
            {
                case JointType.SlideAlongX:
                    return Pose.AlongX(q);
                case JointType.SlideAlongY:
                    return Pose.AlongY(q);
                case JointType.Revolute:
                    return Pose.AboutZ(q);
                default:
                    throw new NotSupportedException("Unknown joint type.");
            }
        }
        /// <summary>
        /// Gets the joint axis.
        /// </summary>
        /// <param name="pose">The joint pose.</param>
        public Vector21 GetAxis(Pose pose)
        {
            switch (Type)
            {
                case JointType.SlideAlongX:
                    return Vector21.Twist(Vector2.UnitX.Rotate(pose.Orientation));
                case JointType.SlideAlongY:
                    return Vector21.Twist(Vector2.UnitY.Rotate(pose.Orientation));
                case JointType.Revolute:
                    return Vector21.Twist(1f, pose.Position);
                default:
                    throw new NotSupportedException("Unknown joint type.");
            }
        }

        #endregion

        public override string ToString()
        {
            return $"[{Index}] Frame: Local={LocalPosition}, Parent#={ParentIndex}";
        }
    }
}
