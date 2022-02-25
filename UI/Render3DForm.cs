using System;
using System.Windows.Forms;
using System.Numerics;
using System.Drawing;

namespace JA.UI
{
    using JA.Numerics;
    using JA.Numerics.Simulation;
    using JA.Numerics.Simulation.Spatial;
    using JA.Numerics.Simulation.Spatial.Geometry;
    using JA.Numerics.Simulation.Spatial.Solvers;

    public partial class Render3DForm : Form
    {
        readonly Camera camera;
        const float pi = (float)Math.PI;
        const float deg = pi / 180;
        bool update = true;
        bool showMesh = false;
        public World3 Scene { get; set; }
        public MbdSolver MbdSolver { get; private set; }
        public ChainSolver ChainSolver { get; private set; }
        public Render3DForm()
        {
            InitializeComponent();

            this.camera = new Camera(pictureBox1, 7f, 0.3f);
            camera.FocalPoint = 0.3f * Vector3.UnitX;
            this.Scene = new World3(UnitSystem.SI);
            this.timer1.Interval = 15;
            this.timer1.Tick += (s, ev) =>
            {
                float h = timer1.Interval / 1000f;
                if (update)
                {
                    MbdSolver?.Update(h);
                    ChainSolver?.Update(h);
                }
                pictureBox1.Invalidate();
                //propertyGrid1.Refresh();
            };
            this.timer1.Start();
            this.camera.PropertyChanged += (s, ev) =>
            {
                if (object.ReferenceEquals(propertyGrid1.SelectedObject, camera))
                {
                    propertyGrid1.Refresh();
                }
            };
            this.Scene.PropertyChanged += (s, ev) =>
            {
                if (object.ReferenceEquals(propertyGrid1.SelectedObject, Scene))
                {
                    propertyGrid1.Refresh();
                }
            };
            this.KeyDown += (s, ev) =>
            {
#pragma warning disable IDE0010 // Add missing cases
                switch (ev.KeyCode)
#pragma warning restore IDE0010 // Add missing cases
                {
                    case Keys.Escape:
                        this.Close();
                        break;
                    case Keys.Space:
                        update = !update;
                        break;
                    case Keys.R:
                        camera.Yaw = 0;
                        camera.Pitch = 0;
                        camera.OnPropertyChanged(nameof(camera.Yaw));
                        camera.OnPropertyChanged(nameof(camera.Pitch));
                        break;
                    case Keys.W:
                        showMesh = !showMesh;
                        break;
                }
            };

            this.camera.Paint += (g, c) =>
            {
                //c.Render(g, Scene, showMesh);
                Scene.Render(g, c, out var scale);
                MbdSolver?.Render(g, c, scale);
                ChainSolver?.Render(g, c, scale);
            };

            pictureBox1.Focus();
        }

        protected void UpdateUI()
        {
            var status = update ? "RUN" : "STOP";
            statusToolStripStatusLabel.Text = $"[{status}] MBD: {MbdSolver?.Bodies.Length ?? 0} Chain: {ChainSolver?.Bodies.Length ?? 0} Geometry: {Scene.GeometryList.Count}";
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            update =false;

            //Demo.DemoSphere(Scene);
            //Demo.Demo4PhoneFlip(Scene);
            var pivot = new Vector3(0f, 0f, 0f);
            float m = 0.5f, L = 0.3f;
            var body = new MassProperties(UnitSystem.SI, m, (2*m)*Matrix3.Diagonal(317.5e-6f, 3750e-6f, 3750e-6f), Vector3.Zero);
            var mesh = Mesh.CreateCube(UnitSystem.SI, Color.Blue, L, L/12, L/12);
            mesh.ElementList[0].Color = Color.Red;
            var localPos = Vector3.UnitX * L;
            var chain = Scene.AddChain(6, mesh, localPos/2, m, localPos);
            camera.SceneSize = 0.9f;
            camera.FocalPoint = 0.5f*Vector3.UnitX - 0.5f*Vector3.UnitY;
            chain.Pivot = pivot;
            chain[0].InitialDisplacement = 0f * deg;
            chain[0].InitialSpeed = -0.6f;
            chain[1].InitialSpeed = -0.35f;  
            
            MbdSolver = Scene.GetMbdSolver();
            ChainSolver = Scene.GetChainSolver(0, false);

            this.MbdSolver.PropertyChanged += (s, ev) =>
            {
                if (object.ReferenceEquals(propertyGrid1.SelectedObject, MbdSolver))
                {
                    propertyGrid1.Refresh();
                }
            };

            this.ChainSolver.PropertyChanged += (s, ev) =>
            {
                if (object.ReferenceEquals(propertyGrid1.SelectedObject, ChainSolver))
                {
                    propertyGrid1.Refresh();
                }
            };

            if (MbdSolver.Bodies.Length > 0)
            {
                propertyGrid1.SelectedObject = MbdSolver;
            }
            if (ChainSolver.Bodies.Length > 0)
            {
                propertyGrid1.SelectedObject = ChainSolver;
            }

            UpdateUI();
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            if (MbdSolver != null)
            {
                propertyGrid1.SelectedObject = MbdSolver;
            }
            else if (ChainSolver != null)
            {
                propertyGrid1.SelectedObject = ChainSolver;
            }
        }

        private void linkLabel2_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            propertyGrid1.SelectedObject = camera;
        }

        private void startToolStripButton_Click(object sender, EventArgs e)
        {
            update = !update;
            UpdateUI();
        }

        private void resetToolStripButton_Click(object sender, EventArgs e)
        {
            update = false;

            MbdSolver?.Reset();
            ChainSolver?.Reset();

            UpdateUI();
        }
    }
}
