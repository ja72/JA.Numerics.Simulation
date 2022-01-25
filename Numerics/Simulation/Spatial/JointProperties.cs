using System.ComponentModel;

namespace JA.Numerics.Simulation.Spatial
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
            this.Type = joint;
            this.Motion = motion;
            this.JointDriver = World3.ZeroDriver;
        }
        public JointProperties(JointType type, Prescribed motion, JointDriver jointDriver) : this(type, motion)
        {
            JointDriver=jointDriver;
        }
        public JointProperties(JointProperties copy)
        {
            this.Type = copy.Type;
            this.Motion = copy.Motion;
            this.JointDriver = copy.JointDriver;
        }
        public static readonly JointProperties AlongX = new JointProperties(JointType.SlideAlongX  );
        public static readonly JointProperties AlongY = new JointProperties(JointType.SlideAlongY  );
        public static readonly JointProperties AlongZ = new JointProperties(JointType.SlideAlongZ  );
        public static readonly JointProperties AboutX = new JointProperties(JointType.RotateAboutX );
        public static readonly JointProperties AboutY = new JointProperties(JointType.RotateAboutY );
        public static readonly JointProperties AboutZ = new JointProperties(JointType.RotateAboutZ );

        public static implicit operator JointProperties(JointType joint) => new JointProperties(joint);

        public JointType Type { get; set; }
        /// <summary>
        /// Gets or sets if the <see cref="JointDriver"/> defines motion or loads.
        /// </summary>
        public Prescribed Motion { get; set; }
        public JointDriver JointDriver { get; set; }
        public bool IsRevolute { get => Type== JointType.RotateAboutX ||Type== JointType.RotateAboutY || Type== JointType.RotateAboutZ; }
        public bool IsPrismatic { get => Type== JointType.SlideAlongX ||Type== JointType.SlideAlongY || Type== JointType.SlideAlongZ; }
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
