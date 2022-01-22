using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Numerics;
using System.Windows.Forms;

namespace JA.Numerics.UI
{
    using JA.Numerics.Simulation;
    using JA.Numerics.Simulation.Spatial;

    public class MouseControl
    {
        public Point DownPos { get; set; }
        public Point UpPos { get; set; }
        public Point CurrentPos { get; set; }
        public MouseButtons Buttons { get; set; }

        public (int dx, int dy) CurrentDelta => (CurrentPos.X - DownPos.X, CurrentPos.Y - DownPos.Y);
        public (int dx, int dy) DragDelta => (UpPos.X - DownPos.X, UpPos.Y - DownPos.Y);
    }

    public delegate void CameraPaintHandler(Camera camera, Graphics g);

    public class Camera
    {
        public event CameraPaintHandler Paint;

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
            LightPos = new Vector3(0 * sceneSize, 0 * sceneSize / 2, -sceneSize);
            Mouse = new MouseControl();
            Yaw = 0;
            Pitch = 0;
            Roll = 0;
            target.Paint += (s, ev) =>
            {
                Paint?.Invoke(this, ev.Graphics);
            };
            target.MouseDown += (s, ev) =>
            {
                Mouse.DownPos = ev.Location;
                Mouse.Buttons = ev.Button;
            };
            target.MouseUp += (s, ev) =>
            {
                Mouse.UpPos = ev.Location;
                Mouse.Buttons = ev.Button;
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
                }
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
            get => Quaternion.Inverse( Quaternion.CreateFromYawPitchRoll(-Yaw, -Pitch, -Roll) );
        }
        public Vector3 LightPos { get; set; }

        public float DrawSize { get => 2 * (float)Math.Tan(FOV / 2 * Math.PI / 180); }
        public Vector3 EyePos { get => Vector3.Transform(Vector3.UnitZ * SceneSize / DrawSize, Quaternion.Inverse(Orientation)); }
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
        public bool IsVisible(Polygon polygon)
            => polygon.Nodes.Length < 3 || IsVisible(polygon.Nodes[0], polygon.Normal);
        /// <summary>
        /// Determines whether a face is visible. 
        /// </summary>
        /// <param name="position">Any position on the face.</param>
        /// <param name="normal">The face normal.</param>
        public bool IsVisible(Vector3 position, Vector3 normal)
        {
            float λ = Vector3.Dot(normal, position - EyePos);

            return λ < 0;
        }

        public void Render(Graphics g, Scene scene, bool triangles = false)
        {
            var gsave = g.Save();
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TranslateTransform(Target.ClientSize.Width / 2f, Target.ClientSize.Height / 2f);
            
            using (var pen = new Pen(Color.Black, 0))
            using (var fill = new SolidBrush(Color.Black))
            {
                var csys = Project(Vector3.Zero, Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ);
                pen.CustomEndCap = new AdjustableArrowCap(0.6f, 0.2f, true);
                pen.Color = Color.Red;
                g.DrawLine(pen, csys[0], csys[1]);
                pen.Color = Color.Green;
                g.DrawLine(pen, csys[0], csys[2]);
                pen.Color = Color.Blue;
                g.DrawLine(pen, csys[0], csys[3]);
                pen.Color = Color.Blue;
                pen.EndCap = LineCap.NoAnchor;
                var light = LightPos.Unit();
                var R = Matrix4x4.CreateFromQuaternion(Quaternion.Inverse(Orientation));
                light = Vector3.TransformNormal(light, R);
                for (int k = 0; k < scene.Bodies.Count; k++)
                {
                    var body = scene.Bodies[k];
                    var state = scene.Current.State[k];
                    if (body.Mesh == null) continue;
                    var mesh = body.Mesh;
                    for (int index = 0; index < mesh.Elements.Count; index++)
                    {
                        var element = mesh.Elements[index];
                        var gp = new GraphicsPath();
                        var poly = mesh.GetPolygon(index, state.Pose);
                        if (triangles)
                        {
                            var faces = poly.GetTriangles();
                            foreach (var trig in faces)
                            {
                                gp.AddPolygon(Project(trig));
                            }
                        }
                        else
                        {
                            gp.AddPolygon(Project(poly));
                        }
                        if (IsVisible(poly))
                        {
                            var (H, S, L) = element.Color.GetHsl();
                            var color = (H, S, L).GetColor(0.5f);
                            fill.Color = color;
                            g.FillPath(fill, gp);
                        }
                        pen.Color = element.Color;
                        g.DrawPath(pen, gp);
                    }
                }
            }
            g.Restore(gsave);
        }
    }
}
