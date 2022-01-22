

namespace JA.Numerics.Simulation.Planar
{
    public delegate float JointDriver(float time, float jointDisplacement, float jointSpeed);

    public enum JointType
    {
        SlideAlongX,
        SlideAlongY,
        Revolute,
    }

    public enum Prescribed
    {
        Motion,
        Load
    }

    public struct JointProperties
    {
        public JointProperties(JointType joint)
            : this(joint, Prescribed.Load) { }
        public JointProperties(JointType joint, Prescribed motion)
        {
            this.Type = joint;
            this.Motion = motion;
        }
        public static implicit operator JointProperties(JointType joint) => new JointProperties(joint);

        public JointType Type { get; }
        public Prescribed Motion { get; }
    }
}
