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
    using Geometry;
    using JA.UI;
    using JA.Numerics.Simulation.Spatial.Solvers;

    public delegate Vector33 AppliedForce(float time, Pose pose, Vector33 velocity);
    public delegate float JointDriver(float time, float jointDisplacement, float jointSpeed);

    /// <summary>
    /// Simulartion World. Contains lists of geometries, single solid bodies, and connected chains.
    /// </summary>
    /// <seealso cref="Solid" />
    /// <seealso cref="Shape" />
    /// <seealso cref="Chain"/>
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class World3 :
        IHasUnits,
        INotifyPropertyChanged
    {
        public static readonly AppliedForce ZeroForce = (t, r, v) => Vector33.Zero;
        public static readonly JointDriver ZeroDriver = (t, q, qp) => 0;

        internal readonly List<Solid> bodyList;
        internal readonly List<Shape> geometryList;
        internal readonly List<Chain> chainList;

        #region Notify Property
        public event PropertyChangedEventHandler PropertyChanged;
        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        #endregion

        public World3(UnitSystem units) : this(units, 
            Unit.Acceleration.Convert(UnitSystem.SI, units) * (-10 * Vector3.UnitY)) { }
        public World3(UnitSystem units, Vector3 gravity)
        {
            Units = units;
            geometryList = new List<Shape>();
            bodyList = new List<Solid>();
            chainList = new List<Chain>();
            Gravity = gravity;
        }

        [Category("Simulation")]
        public Vector3 Gravity { get; set; }
        [Category("Model")]
        public IReadOnlyList<Shape> GeometryList { get => geometryList; }

        [Category("Model")]
        public IReadOnlyList<Solid> BodyList { get => bodyList; }

        [Category("Model")]
        public IReadOnlyList<Chain> ChainList { get => chainList; }

        [Category("Model")]
        public UnitSystem Units { get; private set; }

        public void ConvertTo(UnitSystem target)
        {
            Gravity *= Unit.Acceleration.Convert(Units, target);
            for (int i = 0; i < bodyList.Count; i++)
            {
                bodyList[i].ConvertTo(target);
            }
            for (int i = 0; i < chainList.Count; i++)
            {
                chainList[i].ConvertTo(target);
            }
            for (int i = 0; i < geometryList.Count; i++)
            {
                geometryList[i].ConvertTo(target);
            }
            Units = target;
        }
        public void AddGeometry(Shape mesh)
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

        public ChainSolver GetChainSolver(int index=0, bool enableContacts = false) 
            => new ChainSolver(this.ChainList[index], enableContacts);
        public ChainSolver[] GetChainSolvers(bool enableConacts) => chainList.Select((item) 
            => new ChainSolver(item, enableConacts)).ToArray();
        public MbdSolver GetMbdSolver() => new MbdSolver(this);


        #region Formatting
        public override string ToString()
            => $"World(Bodies={bodyList.Count}, Chain={chainList.Count}, Geometry={geometryList.Count} Gravity={Gravity})";
        #endregion

        public void Render(Graphics g, Camera camera, out float scale)
        {
            camera.SetupView(g, out scale);

            //scale *= Unit.Length.Factor(Units);
            camera.RenderCsys(g, scale);

            for (int w = 0; w < GeometryList.Count; w++)
            {
                var mesh = GeometryList[w];
                mesh.Render(g, camera, Pose.Origin);
            }
        }

    }
}
