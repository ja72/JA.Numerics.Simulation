using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Numerics;
using System.Windows.Forms;
using JA.Numerics.Simulation.Planar;
using JA.Numerics.UI;

namespace JA.Numerics.Simulation.Spatial
{
    public delegate Vector33 AppliedForce(float time, Pose pose, Vector33 velocity);
    public delegate float JointDriver(float time, float jointDisplacement, float jointSpeed);


    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class World3 : IVisible,
        IHasUnits<World3>, 
        INotifyPropertyChanged
    {
        public static readonly AppliedForce ZeroForce = (t, r, v) => Vector33.Zero;
        public static readonly JointDriver ZeroDriver = (t, q, qp) => 0;

        internal readonly List<Solid> bodyList;
        internal readonly List<Geometry> geometryList;
        internal readonly List<Link3> linkList;

        public event PropertyChangedEventHandler PropertyChanged;

        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        public World3(UnitSystem units) : this(units, Vector3.Zero) { }
        public World3(UnitSystem units, Vector3 gravity)
        {
            Units = units;
            bodyList = new List<Solid>();
            geometryList = new List<Geometry>();
            linkList = new List<Link3>();
            Gravity = gravity;
        }

        World3(UnitSystem units, Vector3 gravity, IEnumerable<Solid> bodies, IEnumerable<Link3> chain, IEnumerable<Geometry> geometry) : this(units)
        {
            Gravity = gravity;
            bodyList.AddRange(bodies);
            geometryList.AddRange(geometry);
            linkList.AddRange(chain);
        }
        [Category("Simulation")]
        public Vector3 Gravity { get; set; }
        [Browsable(false)] public IList<Geometry> GeometryList => geometryList;

        [Category("Model")] public Geometry[] Geometries { get => GeometryList.ToArray(); }
        [Browsable(false)] public IList<Solid> BodyList => bodyList;

        [Category("Model")] public Solid[] Bodies { get => BodyList.ToArray(); }
        [Browsable(false)] public IList<Link3> Links { get => linkList; }
        [Category("Model")] public Link3[] Chains { get => linkList.ToArray(); }

        [Category("Model")]
        public UnitSystem Units { get; }

        public void AddGeometry(Geometry mesh)
        {
            geometryList.Add(mesh);
        }

        public void AddBody(Solid solid)
        {
            bodyList.Add(solid);
        }
        internal void AddLink(Link3 frame)
        {
            linkList.Add(frame);
        }
        public Link3 AddChild(Mesh mesh, Pose meshPosition, float mass, Pose onWorld, JointProperties type, float initialPosition = 0)
            => AddChild(mesh, meshPosition, mesh.GetMmoiFromMass(mass), onWorld, type, initialPosition);
        public Link3 AddChild(Mesh mesh, Pose meshPosition, MassProperties mass, Pose onWorld, JointProperties type, float initialPosition = 0)
            => new Link3(mesh, this, meshPosition, mass, onWorld, type, initialPosition);

        public Link3 AddChain(int count, Mesh mesh, Vector3 meshPosition, float mass, Pose onParent)
            => AddChain(count, mesh, meshPosition, mesh.GetMmoiFromMass(mass), onParent);
        public Link3 AddChain(int count, Mesh mesh, Vector3 meshPosition, MassProperties bodyProperty, Pose onParent)
        {
            JointDriver driver = (t, q, qp) => -0.02f * qp;
            if (count == 0) return null;
            var joint = new JointProperties(JointType.RotateAboutZ, Prescribed.Load, driver);
            var item = AddChild(mesh, meshPosition, bodyProperty, Pose.Origin, joint);
            for (int i = 1; i < count; i++)
            {
                item = item.AddChild(mesh, meshPosition, bodyProperty, onParent, joint);
            }
            return item;
        }

        public Link3 SlideAlongX(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialDisplacement = 0)
            => AddChild(mesh, localPosition, mass, onWorld, JointType.SlideAlongX, initialDisplacement);
        public Link3 SlideAlongY(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialDisplacement = 0)
            => AddChild(mesh, localPosition, mass, onWorld, JointType.SlideAlongY, initialDisplacement);
        public Link3 AddRevolute(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialAngle = 0)
            => AddChild(mesh, localPosition, mass, onWorld, JointType.RotateAboutZ, initialAngle);
        public Link3 AddCollarAlongX(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(null, localPosition, MassProperties.Zero, onWorld, JointType.SlideAlongX, initialDisplacement)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);
        public Link3 AddCollarAlongY(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(null, localPosition, MassProperties.Zero, onWorld, JointType.SlideAlongY, initialDisplacement)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);
        public Link3 AddFree(Mesh mesh, Pose localPosition, MassProperties mass, Pose onWorld, float initialX, float intialY, float initialAngle)
            => AddChild(null, localPosition, MassProperties.Zero, onWorld, JointType.SlideAlongX, initialX)
                    .AddChild(null, Pose.Origin, MassProperties.Zero, Pose.Origin, JointType.SlideAlongY, intialY)
                    .AddChild(mesh, Pose.Origin, mass, Pose.Origin, JointType.RotateAboutZ, initialAngle);

        public void RemoveChild(Link3 child)
        {
            foreach (var item in linkList.Where(f => f.Parent == child))
            {
                item.SetParent(null);
            }
            linkList.Remove(child);
        }

        public ChainSolver GetChainSolver() => new ChainSolver(this);
        public MbdSolver GetMbdSolver() => new MbdSolver(this);


        public World3 ConvertTo(UnitSystem target)
        {
            float fl = UnitFactors.Length(Units, target);
            return new World3(target,
                fl*Gravity,
                bodyList.Select((item) => item.ConvertTo(target)),
                linkList.Select((item) => item.ConvertTo(target)),
                geometryList.Select((item) => item.ConvertTo(target)));
        }

        #region Formatting
        public override string ToString()
            => $"World(Bodies={bodyList.Count}, Chain={linkList.Count}, Geometry={geometryList.Count} Gravity={Gravity})";
        #endregion

        public void Render(Graphics g, Camera camera)
        {
            camera.SetupView(g);

            for (int w = 0; w < GeometryList.Count; w++)
            {
                var mesh = GeometryList[w];
                mesh.Render(g, camera);
            }
        }

    }
}
