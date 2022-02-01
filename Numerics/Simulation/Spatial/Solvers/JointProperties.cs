using System;
using System.ComponentModel;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial.Solvers
{

    public enum JointType
    {
        SlideAlongX,
        SlideAlongY,
        SlideAlongZ,
        RotateAboutX,
        RotateAboutY,
        RotateAboutZ,
    }

    public enum Prescribed
    {
        Motion,
        Load
    }
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class JointProperties : ICanConvert<JointProperties>
    {

        public JointProperties(JointType joint)
            : this(joint, Prescribed.Load) { }
        public JointProperties(JointType joint, Prescribed motion)
        {
            Type = joint;
            Motion = motion;
            JointDriver = World3.ZeroDriver;
        }
        public JointProperties(JointType type, Prescribed motion, JointDriver jointDriver) : this(type, motion)
        {
            JointDriver=jointDriver;
        }
        public JointProperties(JointProperties copy)
        {
            Type = copy.Type;
            Motion = copy.Motion;
            JointDriver = copy.JointDriver;
        }
        public static readonly JointProperties AlongX = new JointProperties(JointType.SlideAlongX);
        public static readonly JointProperties AlongY = new JointProperties(JointType.SlideAlongY);
        public static readonly JointProperties AlongZ = new JointProperties(JointType.SlideAlongZ);
        public static readonly JointProperties AboutX = new JointProperties(JointType.RotateAboutX);
        public static readonly JointProperties AboutY = new JointProperties(JointType.RotateAboutY);
        public static readonly JointProperties AboutZ = new JointProperties(JointType.RotateAboutZ);

        public static implicit operator JointProperties(JointType joint) => new JointProperties(joint);

        public JointType Type { get; set; }
        /// <summary>
        /// Gets or sets if the <see cref="JointDriver"/> defines motion or loads.
        /// </summary>
        public Prescribed Motion { get; set; }
        public JointDriver JointDriver { get; set; }
        public bool IsRevolute { get => Type== JointType.RotateAboutX ||Type== JointType.RotateAboutY || Type== JointType.RotateAboutZ; }
        public bool IsPrismatic { get => Type== JointType.SlideAlongX ||Type== JointType.SlideAlongY || Type== JointType.SlideAlongZ; }

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
        public Vector3 GetDirection(Pose pose)
        {
            switch (Type)
            {
                case JointType.SlideAlongX:
                case JointType.RotateAboutX:
                    return Vector3.UnitX.Rotate(pose.Orientation);
                case JointType.SlideAlongY:
                case JointType.RotateAboutY:
                    return Vector3.UnitY.Rotate(pose.Orientation);
                case JointType.SlideAlongZ:
                case JointType.RotateAboutZ:
                    return Vector3.UnitZ.Rotate(pose.Orientation);
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
            var direction = GetDirection(pose);
            switch (Type)
            {
                case JointType.SlideAlongX:
                    return Vector33.Twist(direction);
                case JointType.SlideAlongY:
                    return Vector33.Twist(direction);
                case JointType.SlideAlongZ:
                    return Vector33.Twist(direction);
                case JointType.RotateAboutX:
                    return Vector33.Twist(direction, pose.Position, 0);
                case JointType.RotateAboutY:
                    return Vector33.Twist(direction, pose.Position, 0);
                case JointType.RotateAboutZ:
                    return Vector33.Twist(direction, pose.Position, 0);
                default:
                    throw new NotSupportedException("Unknown joint type.");
            }
        }

        #endregion

        public JointProperties ConvertFromTo(UnitSystem units, UnitSystem target)
        {
            var driver = JointDriver;
            if (driver != null && driver != World3.ZeroDriver)
            {
                float fl = UnitFactors.Length(units, target);
                float ff = UnitFactors.Force(units, target);
                float ft = UnitFactors.Torque(units, target);


                if (Motion == Prescribed.Load)
                {
                    (float f_in, float f_out) = IsPrismatic ? (1/fl, ff) : (1, ft);
                    driver = (t, q, qp) => f_out * JointDriver(t, f_in*q, f_in*qp);
                }
                else
                {
                    (float f_in, float f_out) = IsPrismatic ? (1/fl, fl) : (1, 1);
                    driver = (t, q, qp) => f_out * JointDriver(t, f_in*q, f_in*qp);
                }
            }
            return new JointProperties(Type, Motion, driver);
        }
        public override string ToString()
        {
            return $"Joint(Type={Type}, Driver={Motion})";
        }
    }
}
