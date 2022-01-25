using System;
using System.ComponentModel;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public readonly struct JointState : ICanConvert<JointState>
    {
        public static readonly JointState Default = new JointState(JointType.RotateAboutZ, 0f, 0f);
        public JointState(JointType type, float angle, float speed=0f) :  this()
        {
            this.Type = type;
            this.Angle = angle;
            this.Speed = speed;
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

        public JointState ConvertFromTo(UnitSystem unit, UnitSystem target)
        {
            float fl = UnitFactors.Length(unit, target);

            switch (Type)
            {
                case JointType.SlideAlongX:
                case JointType.SlideAlongY:
                case JointType.SlideAlongZ:
                    return new JointState(Type, fl*Angle, fl*Speed);
                case JointType.RotateAboutX:
                case JointType.RotateAboutY:
                case JointType.RotateAboutZ:
                    return new JointState(Type, Angle, Speed);
                default:
                    throw new NotSupportedException(nameof(Type));
            }
        }
        public override string ToString()
        {
            return $"{Type}(q={Angle}, qp={Speed})";
        }
    }
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public struct JointInfo : ICanConvert<JointInfo>
    {
        public static readonly JointInfo Default = new JointInfo(0f, 0f, 0f, 0f, JointProperties.AboutZ);
        public JointInfo(JointProperties joint, JointState state, float acceleration, float torque)
        {
            this.Joint = joint;
            this.Angle = state.Angle;
            this.Speed = state.Speed;
            this.Acceleration = acceleration;
            this.Torque = torque;
        }
        public JointInfo(JointProperties joint, float time, JointState state, float value=0)
        {
            this.Joint = joint;
            this.Angle = state.Angle;
            this.Speed = state.Speed;
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

        public JointInfo ConvertFromTo(UnitSystem unit, UnitSystem target)
        {
            float fl = UnitFactors.Length(unit, target);
            float ff = UnitFactors.Force(unit, target);
            float ft = UnitFactors.Torque(unit, target);

            switch (Joint.Type)
            {
                case JointType.SlideAlongX:
                case JointType.SlideAlongY:
                case JointType.SlideAlongZ:
                    return new JointInfo(fl*Angle, fl*Speed, fl*Acceleration, ff*Torque, Joint.ConvertFromTo(unit, target));
                case JointType.RotateAboutX:
                case JointType.RotateAboutY:
                case JointType.RotateAboutZ:
                    return new JointInfo(Angle, Speed, Acceleration, ft*Torque, Joint.ConvertFromTo(unit, target));
                default:
                    throw new NotSupportedException(nameof(Joint.Type));
            }
        }

        public JointState GetRate() => new JointState(Joint.Type, Speed, Acceleration);

        public override string ToString()
        {
            return $"{Joint.Type}(q={Angle}, qp={Speed}, qpp={Acceleration}, Q={Torque})";
        }
    }
}
