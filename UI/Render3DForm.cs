using System;
using System.Windows.Forms;
using System.Numerics;

namespace JA.UI
{
    using JA.Numerics.Simulation;
    using JA.Numerics.Simulation.Spatial;
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

            this.camera = new Camera(pictureBox1, 7f, 5f);
            this.Scene = new World3(UnitSystem.MMKS);
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
                Scene.Render(g, c);
                MbdSolver?.Render(g, c);
                ChainSolver?.Render(g, c);
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

            //Demo.DemoSphere(Scene);
            //Demo.Demo4PhoneFlip(Scene);
            var pivot = new Vector3(-2f, 2f, 0f);
            Demo.ChainDemo(Scene, 4, pivot);
            Scene.ChainList[0].Links[0].InitialDisplacement = -90f.Deg();
            Scene.ChainList[0].Links[0].InitialSpeed = 2f;

            MbdSolver = Scene.GetMbdSolver();
            ChainSolver = Scene.GetChainSolver();

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
