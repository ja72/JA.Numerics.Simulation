using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics.Simulation
{

    using JA.Numerics.Simulation.Spatial;
    using JA.Numerics.Simulation.Spatial.Solvers;

    public static class Demo
    {
        public static void DemoSphere(World3 scene)
        {
            var mesh = Mesh.CreateSphere(Color.SteelBlue, 2f);

            var solid1 = new Solid(mesh, scene, 0.1f, -5f*Vector3.UnitX);
            solid1.SetMotion(Vector3.Zero, 4f * Vector3.UnitX);

            var solid2 = new Solid(mesh, scene, 0.1f, 0f*Vector3.UnitX);
            solid2.SetMotion(Vector3.Zero, 4f * Vector3.UnitY);

            var solid3 = new Solid(mesh, scene, 0.1f, 5f*Vector3.UnitX);
            solid3.SetMotion(Vector3.Zero, 4f * Vector3.UnitZ);

            scene.AddBody(solid1);
            scene.AddBody(solid2);
            scene.AddBody(solid3);
        }

        public static void Demo4PhoneFlip(World3 scene)
        {
            var cg = 0f*Vector3.UnitY;
            var mesh = Mesh.CreateCube(Color.Blue, 4f, 0.6f, 2.6f);
            mesh.ApplyTranform(cg);
            mesh.ElementList[0].Color = Color.Red;
            mesh.ElementList[1].Color = Color.Green;

            for (int i = 0; i < 4; i++)
            {
                var pose = Pose.At((i-1.5f) * 3f * Vector3.UnitX);
                var solid = new Solid(mesh, scene, 0.1f, pose);
                var axis = Vector3.Normalize(Vector3.UnitZ + (2e-7f*(i-1.5f))*Vector3.UnitX);
                solid.SetMotion(Vector33.Twist(4f * axis, solid.CG, 0));
                scene.AddBody(solid);
            }
        }
        public static Chain ChainDemo(World3 scene, int n, Vector3 pivot)
        {
            scene.Gravity = -10 * Vector3.UnitY;
            float m = 0.15f, L = 1.50f, h = .650f;
            var mesh = Mesh.CreateCube(Color.Blue, L, h, h);
            mesh.ElementList[0].Color = Color.Red;
            var localPos = Vector3.UnitX * L;
            var body = mesh.GetMmoiFromMass(m);
            var chain = scene.AddChain(n, mesh, localPos/2, m, localPos);
            chain.Pivot = pivot;
            return chain;
        }

    }
}
