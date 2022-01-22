using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using JA.Numerics.Simulation.Spatial;

namespace JA.Numerics.UI
{
    public partial class Render3DForm : Form
    {
        readonly Camera camera;
        const float pi = (float)Math.PI;
        const float deg = pi / 180;
        bool update = true;
        public Scene Scene { get; set; }

        public Render3DForm()
        {
            InitializeComponent();

            this.camera = new Camera(pictureBox1, 7f, 4f);
            this.Scene = new Scene();
            this.timer1.Interval = 15;
            this.timer1.Tick += (s, ev) =>
            {
                float h = timer1.Interval / 1000f;
                if (update) { Scene.Update(h); }
                pictureBox1.Invalidate();
            };
            this.timer1.Start();

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
                }
            };

            this.camera.Paint += (c, g) =>
            {
                c.Render(g, Scene);
            };
        }
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            for (int i = 0; i < 4; i++)
            {
                var mesh = Mesh.CreateCube(Color.Blue, 4f, 0.6f, 2.2f);
                var cg = 0f*Vector3.UnitY;
                mesh.ApplyTranform(cg);
                mesh.Elements[0].Color = Color.Red;
                var pose = Pose.At((i-1.5f) * 3f * Vector3.UnitX);
                var solid = new Solid(Scene, 0.1f, mesh, pose);
                cg = Pose.FromLocal(pose, cg);
                var axis = Vector3.Normalize(Vector3.UnitZ + (1e-6f*i)*Vector3.UnitX);
                solid.SetMotion(Vector33.Twist(4f * axis, cg, 0));
                Scene.Bodies.Add(solid);
            }

            Scene.Reset();
        }
    }
}
