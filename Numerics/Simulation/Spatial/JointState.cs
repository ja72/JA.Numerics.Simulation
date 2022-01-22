using System;

namespace JA.Numerics.Simulation.Spatial
{
    public struct JointState
    {
        public JointState(float angle, float speed=0f) : this(angle, speed, 0f, 0f) { }
        public JointState(float angle, float speed, float value, Prescribed prescribed) : this()
        {
            this.Angle = angle;
            this.Speed = speed;
            switch (prescribed)
            {
                case Prescribed.Motion:
                    Aceleration = value;
                    Torque = 0;
                    break;
                case Prescribed.Load:
                    Torque = value;
                    Aceleration = 0;
                    break;
                default:
                    throw new NotSupportedException();
            }
        }
        public JointState(float angle, float speed, float acceleration, float torque) : this()
        {
            this.Angle = angle;
            this.Speed = speed;
            this.Aceleration = acceleration;
            this.Torque = torque;
        }

        public float Angle { get; }
        public float Speed { get; }
        public float Aceleration { get; set; }
        public float Torque { get; set; }

        public static readonly JointState Default = new JointState(0f, 0f, 0f, 0f);
    }
}
