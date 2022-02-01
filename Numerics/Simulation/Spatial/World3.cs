using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Numerics;
using System.Windows.Forms;

namespace JA.Numerics.Simulation.Spatial
{
    using JA.UI;
    using JA.Numerics.Simulation.Spatial.Solvers;

    public delegate Vector33 AppliedForce(float time, Pose pose, Vector33 velocity);
    public delegate float JointDriver(float time, float jointDisplacement, float jointSpeed);


    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class World3 :
        IHasUnits<World3>,
        INotifyPropertyChanged
    {
        public static readonly AppliedForce ZeroForce = (t, r, v) => Vector33.Zero;
        public static readonly JointDriver ZeroDriver = (t, q, qp) => 0;

        internal readonly List<Solid> bodyList;
        internal readonly List<Geometry> geometryList;
        internal readonly List<Chain> chainList;

        #region Notify Property
        public event PropertyChangedEventHandler PropertyChanged;
        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        #endregion

        public World3(UnitSystem units) : this(units, Vector3.Zero) { }
        public World3(UnitSystem units, Vector3 gravity)
        {
            Units = units;
            geometryList = new List<Geometry>();
            bodyList = new List<Solid>();
            chainList = new List<Chain>();
            Gravity = gravity;
        }

        World3(World3 copy, UnitSystem target)
        {
            Units = target;
            bodyList.AddRange(copy.bodyList.Select(item => item.ConvertTo(target)));
            chainList.AddRange(copy.chainList.Select(item => item.ConvertTo(target)));
            geometryList.AddRange(copy.geometryList.Select(item => item.ConvertTo(target)));
            Gravity =  copy.Gravity * Unit.Acceleration.Convert(copy.Units, target);
        }
        [Category("Simulation")]
        public Vector3 Gravity { get; set; }
        [Category("Model")]
        public IReadOnlyList<Geometry> GeometryList { get => geometryList; }

        [Category("Model")]
        public IReadOnlyList<Solid> BodyList { get => bodyList; }

        [Category("Model")]
        public IReadOnlyList<Chain> ChainList { get => chainList; }

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

        public Chain AddChain(int count, Mesh mesh, Vector3 meshPosition, float mass, Pose onParent)
            => AddChain(count, mesh, meshPosition, mesh.GetMmoiFromMass(mass), onParent);
        public Chain AddChain(int count, Mesh mesh, Vector3 meshPosition, MassProperties bodyProperty, Pose onParent)
        {
            static float driver(float t, float q, float qp) => -0.02f * qp - 0.1f*q;
            if (count == 0) return null;
            var joint = new JointProperties(JointType.RotateAboutZ, Prescribed.Load, driver);
            var body = new Body(mesh, this, bodyProperty, meshPosition);
            var chain = new Chain(count, body, onParent, joint);
            chainList.Add(chain);
            return chain;
        }

        public ChainSolver GetChainSolver(int index=0) => new ChainSolver(this.ChainList[index]);
        public ChainSolver[] GetChainSolvers() => chainList.Select((item) => new ChainSolver(item)).ToArray();
        public MbdSolver GetMbdSolver() => new MbdSolver(this);

        public World3 ConvertTo(UnitSystem target)
        {
            return new World3(this, target);
        }

        #region Formatting
        public override string ToString()
            => $"World(Bodies={bodyList.Count}, Chain={chainList.Count}, Geometry={geometryList.Count} Gravity={Gravity})";
        #endregion

        public void Render(Graphics g, Camera camera)
        {
            camera.SetupView(g);

            for (int w = 0; w < GeometryList.Count; w++)
            {
                var mesh = GeometryList[w];
                mesh.Render(g, camera, Pose.Origin);
            }
        }

    }
}
