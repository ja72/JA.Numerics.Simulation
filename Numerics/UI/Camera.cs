using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Numerics;
using System.Windows.Forms;

namespace JA.Numerics.UI
{
    using System.ComponentModel;
    using JA.Numerics.Simulation;
    using JA.Numerics.Simulation.Spatial;

    public interface IVisible
    {
        void Render(Graphics g, Camera camera);
    }

    public delegate void CameraPaintHandler(Graphics g, Camera camera);

    [TypeConverter(typeof(ExpandableObjectConverter))]
    public sealed class Camera : INotifyPropertyChanged
    {
        public event CameraPaintHandler Paint;
        public event PropertyChangedEventHandler PropertyChanged;

        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        /// <summary>
        /// Initializes a new instance of the <see cref="Camera" /> class.
        /// </summary>
        /// <param name="target">The target control to draw scene.</param>
        /// <param name="fov">
        /// The FOV angle (make zero for orthographic projection).
        /// </param>
        /// <param name="sceneSize">Size of the scene across/</param>
        public Camera(Control target, float fov, float sceneSize = 1f)
        {
            Target = target;
            FOV = fov;
            SceneSize = sceneSize;
            LightPosition = new Vector3(1f * sceneSize, 1f * sceneSize, 2f* sceneSize);
            Mouse = new MouseControl();
            Yaw = 0;
            Pitch = 0;
            Roll = 0;
            target.Paint += (s, ev) =>
            {
                var gs = ev.Graphics.Save();
                Paint?.Invoke(ev.Graphics, this);
                ev.Graphics.Restore(gs);
            };
            target.MouseDown += (s, ev) =>
            {
                Mouse.DownPos = ev.Location;
                Mouse.Buttons = ev.Button;
                OnPropertyChanged(nameof(Mouse));
            };
            target.MouseUp += (s, ev) =>
            {
                Mouse.UpPos = ev.Location;
                Mouse.Buttons = ev.Button;
                OnPropertyChanged(nameof(Mouse));
            };
            target.MouseMove += (s, ev) =>
            {
                Mouse.CurrentPos = ev.Location;
                Mouse.Buttons = ev.Button;

                if (ev.Button == MouseButtons.Right)
                {
                    (int dx, int dy) = Mouse.CurrentDelta;
                    float f = 5 * ProjectionScale / ViewSize * (2f - ProjectionScale);
                    Yaw += f * dx;
                    Pitch += f * dy;
                    Mouse.DownPos = Mouse.CurrentPos;
                    //OnPropertyChanged(nameof(Yaw));
                    //OnPropertyChanged(nameof(Pitch));
                }
                OnPropertyChanged(nameof(Mouse));
            };
            target.FindForm().MouseWheel += (s, ev) =>
            {
                var λ = (float)Math.Exp(-ev.Delta/960f);
                SceneSize *= λ;
                OnPropertyChanged(nameof(SceneSize));
            };
        }
        public float Yaw { get; set; }
        public float Pitch { get; set; }
        public float Roll { get; set; }
        public MouseControl Mouse { get; }
        public Control Target { get; }
        public float SceneSize { get; set; }
        public float FOV { get; set; }
        public Quaternion Orientation
        {
            get => Quaternion.Inverse(Quaternion.CreateFromYawPitchRoll(-Yaw, -Pitch, -Roll));
        }
        public Vector3 LightPosition { get; set; }

        public float DrawSize { get => 2 * (float)Math.Tan(FOV / 2 * Math.PI / 180); }
        public Vector3 EyePosition { get => Vector3.Transform(Vector3.UnitZ * SceneSize / DrawSize, Quaternion.Inverse(Orientation)); }
        public PointF[] Project(Triangle triangle) => Project(triangle.A, triangle.B, triangle.C);
        public PointF[] Project(Polygon polygon) => Project(polygon.Nodes);

        public int ViewSize
        {
            get
            {
                int wt = Target.ClientSize.Width - Target.Margin.Left - Target.Margin.Right;
                int ht = Target.ClientSize.Height - Target.Margin.Top - Target.Margin.Bottom;
                return Math.Min(wt, ht);
            }
        }
        /// <summary>
        /// Gets the distance the camera eye needs to be to have field
        /// of view <see cref="FOV"/> for the <see cref="SceneSize"/>.
        /// </summary>
        public float EyeDistance { get => SceneSize / ProjectionScale; }
        /// <summary>
        /// Gets the size of an object 1 unit from the camera that eclipses
        /// The scene size (located at the eye distance away).
        /// </summary>
        public float ProjectionScale { get => 2 * (float)Math.Tan(FOV / 2 * Math.PI / 180); }
        /// <summary>
        /// Projects the specified nodes into a 2D canvas by applied the camera 
        /// orientation and projection.
        /// </summary>
        /// <param name="nodes">The nodes to project.</param>
        /// <returns>A list of Gdi points</returns>
        public PointF[] Project(params Vector3[] nodes)
        {
            float r = ProjectionScale;
            float L = EyeDistance;
            int sz = ViewSize;
            var R = Matrix4x4.CreateFromQuaternion(Orientation);

            var points = new PointF[nodes.Length];
            for (int i = 0; i < points.Length; i++)
            {
                var point = Vector3.Transform(nodes[i], R);
                points[i] = new PointF(
                    +sz / 2 * point.X / (r * (L - point.Z)),
                    -sz / 2 * point.Y / (r * (L - point.Z)));
            }

            return points;
        }
        /// <summary>
        /// Determines whether a face is visible. 
        /// </summary>
        public bool IsVisible(Polygon polygon)
            => polygon.Nodes.Length < 3 || IsVisible(polygon.Nodes[0], polygon.Normal);
        /// <summary>
        /// Determines whether a face is visible. 
        /// </summary>
        /// <param name="position">Any position on the face.</param>
        /// <param name="normal">The face normal.</param>
        public bool IsVisible(Vector3 position, Vector3 normal)
        {
            float λ = Vector3.Dot(normal, position - EyePosition);

            return λ < 0;
        }
        public float DiffuseLight(Polygon polygon) => DiffuseLight(polygon.Nodes[0], polygon.Normal);
        public float DiffuseLight(Vector3 position, Vector3 normal)
        {
            var lightDir = LightPosition.Unit();
            float d = Vector3.Distance(position, LightPosition);
            float λ = Vector3.Dot(normal, lightDir);
            //λ = λ.Cap(0, 1);
            return λ * EyeDistance/(d*d);
        }
        public float SpecularLight(Polygon polygon) => SpecularLight(polygon.Nodes[0], polygon.Normal);
        public float SpecularLight(Vector3 position, Vector3 normal)
        {
            // https://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_reflection_model
            var lightDir = LightPosition.Unit();
            var viewDir = EyePosition.Unit();
            var half = Vector3.Normalize(lightDir + viewDir);
            float λ = Vector3.Dot(normal, half);
            return (float)Math.Pow(λ.Cap(0, 1), 155f);
        }

        void RenderCsys(Graphics g)
        {
            using (var pen = new Pen(Color.Black, 0))
            {
                var csys = Project(Vector3.Zero, Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ);
                pen.CustomEndCap = new AdjustableArrowCap(0.6f, 0.2f, true);
                pen.Color = Color.Red;
                g.DrawLine(pen, csys[0], csys[1]);
                pen.Color = Color.Green;
                g.DrawLine(pen, csys[0], csys[2]);
                pen.Color = Color.Blue;
                g.DrawLine(pen, csys[0], csys[3]);
            }
        }
        public void SetupView(Graphics g)
        {
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TranslateTransform(Target.ClientSize.Width / 2f, Target.ClientSize.Height / 2f);

            RenderCsys(g);
        }

    }
}
