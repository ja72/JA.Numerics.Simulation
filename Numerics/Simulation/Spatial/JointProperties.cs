namespace JA.Numerics.Simulation.Spatial
{
    public delegate float JointDriver(float time, float jointDisplacement, float jointSpeed);

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

    public class JointProperties 
    {
        public static readonly JointDriver ZeroDriver = (t, q, qp) => 0;

        public JointProperties(JointType joint)
            : this(joint, Prescribed.Load) { }
        public JointProperties(JointType joint, Prescribed motion)
        {
            this.JointDriver = ZeroDriver;
            this.Type = joint;
            this.Motion = motion;
        }
        public static implicit operator JointProperties(JointType joint) => new JointProperties(joint);

        public JointType Type { get; set; }
        public Prescribed Motion { get; set; }
        public JointDriver JointDriver { get; set; }
    }
}
