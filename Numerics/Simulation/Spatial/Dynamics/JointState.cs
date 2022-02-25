using System;
using System.ComponentModel;

namespace JA.Numerics.Simulation.Spatial.Solvers
{
    /// <summary>
    /// Single link state describing the joint angle and speed.
    /// </summary>
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public readonly struct JointState 
    {
        public static readonly JointState Default = new JointState(JointType.RotateAboutZ, 0f, 0f);
        public JointState(JointType type, float angle, float speed = 0f) : this()
        {
            Type = type;
            Angle = angle;
            Speed = speed;
        }

        public static implicit operator JointState(JointInfo info)
            => new JointState(info.Joint.Type, info.Angle, info.Speed);

        public JointType Type { get; }
        public float Angle { get; }
        public float Speed { get; }

        #region Algebra
        public static JointState Add(JointState A, JointState B)
        {
            if (A.Type!=B.Type) throw new ArgumentException(nameof(B));
            return new JointState(A.Type,
                A.Angle+B.Angle,
                A.Speed+B.Speed);
        }
        public static JointState Subtract(JointState A, JointState B)
        {
            if (A.Type!=B.Type) throw new ArgumentException(nameof(B));
            return new JointState(A.Type,
                A.Angle-B.Angle,
                A.Speed-B.Speed);
        }

        public static JointState Scale(float factor, JointState A)
        {
            return new JointState(A.Type,
                factor*A.Angle,
                factor*A.Speed);
        }
        #endregion

        #region Operators
        public static JointState operator +(JointState a, JointState b) => Add(a, b);
        public static JointState operator -(JointState a) => Scale(-1, a);
        public static JointState operator -(JointState a, JointState b) => Subtract(a, b);
        public static JointState operator *(float a, JointState b) => Scale(a, b);
        public static JointState operator *(JointState a, float b) => Scale(b, a);
        public static JointState operator /(JointState a, float b) => Scale(1 / b, a);
        #endregion

        public override string ToString()
        {
            return $"{Type}(q={Angle}, qp={Speed})";
        }
    }
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public struct JointInfo 
    {
        public static readonly JointInfo Default = new JointInfo(0f, 0f, 0f, 0f, JointProperties.AboutZ);
        public JointInfo(JointProperties joint, JointState state, float acceleration, float torque)
        {
            Joint = joint;
            Angle = state.Angle;
            Speed = state.Speed;
            Acceleration = acceleration;
            Torque = torque;
        }
        public JointInfo(JointProperties joint, float time, JointState state, float value = 0)
        {
            Joint = joint;
            Angle = state.Angle;
            Speed = state.Speed;
            switch (joint.Motion)
            {
                case Prescribed.Motion:
                    Acceleration = joint.JointDriver(time, state.Angle, state.Speed);
                    Torque = value;
                    break;
                case Prescribed.Load:
                    Torque = joint.JointDriver(time, state.Angle, state.Speed);
                    Acceleration = value;
                    break;
                default:
                    throw new NotSupportedException(joint.Motion.ToString());
            }
        }
        public JointInfo(float angle, float speed, float aceleration, float torque, JointProperties joint)
        {
            Angle=angle;
            Speed=speed;
            Acceleration=aceleration;
            Torque=torque;
            Joint=joint;
        }

        public float Angle { get; }
        public float Speed { get; }
        public float Acceleration { get; set; }
        public float Torque { get; set; }
        public JointProperties Joint { get; }

        public JointState GetRate() => new JointState(Joint.Type, Speed, Acceleration);

        public override string ToString()
        {
            return $"{Joint.Type}(q={Angle}, qp={Speed}, qpp={Acceleration}, Q={Torque})";
        }
    }
}
