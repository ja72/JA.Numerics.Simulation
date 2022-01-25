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

    public static class Demo
    {
        public static void DemoSphere(World3 scene)
        {
            var mesh = Mesh.CreateSphere(Color.SteelBlue, 2f);

            var solid1 = new Solid(mesh, scene, 0.1f, -5f*Vector3.UnitX);
            solid1.SetMotion(Vector3.Zero, 4f * Vector3.UnitX);

            var solid2 = new Solid(mesh, scene, 0.1f,  0f*Vector3.UnitX);
            solid2.SetMotion(Vector3.Zero, 4f * Vector3.UnitY);

            var solid3 = new Solid(mesh, scene, 0.1f,  5f*Vector3.UnitX);
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
        public static void ChainDemo(World3 scene)
        {
            Console.WriteLine("=== Spatial Dynamics Demo ===");
            UnitSystem units = UnitSystem.MMKS;
            scene.Gravity = -10 * Vector3.UnitY;
            Console.WriteLine($"UNITS: {units}");
            float m = 0.15f, L = 1.50f, h = .650f;
            var mesh = Mesh.CreateCube(Color.Blue, L, h, h);
            mesh.ElementList[0].Color = Color.Red;
            var localPos = Vector3.UnitX * L;
            var body = mesh.GetMmoiFromMass(m); 
            Console.WriteLine($"{body}");
            scene.AddChain(6, mesh, localPos/2, m, localPos);
            scene.Chains[0].LocationOnParent = new Vector3(-2f, 2f, 0f);
            Console.WriteLine($"{scene}");

            var sim = scene.GetChainSolver();

            var next = sim.CalcDynamics(out var kin);

            var x = kin.Select((k) => k.Position.Position.X).ToArray();
            var cg = kin.Select((k) => k.CgPosition.X).ToArray();
            var vcy = kin.Select((k) => k.Velocity.TwistAt(k.CgPosition).Y).ToArray();
            var acy = kin.Select((k) => k.CgAccleration.Y).ToArray();

            var qpp = next.Select((st) => st.Acceleration).ToArray();

            Console.WriteLine($"x = {string.Join(",", x)}");
            Console.WriteLine($"cg = {string.Join(",", cg)}");
            Console.WriteLine($"vcy = {string.Join(",", vcy)}");
            Console.WriteLine($"acy = {string.Join(",", acy)}");
            Console.WriteLine($"qpp = {string.Join(",", qpp)}");

            var frc = sim.CheckForceBalance(kin);

            for (int i = 0; i < sim.Bodies.Length; i++)
            {
                Console.WriteLine($"{sim.Bodies[i]}");
                Console.WriteLine($"\tq({i}) = {next[i].Angle}");
                Console.WriteLine($"\tqp({i}) = {next[i].Speed}");
                Console.WriteLine($"\tqpp({i}) = {next[i].Acceleration}");
                Console.WriteLine($"\ttau({i}) = {next[i].Torque}");
                Console.WriteLine($"\tr[{i}] = {kin[i].Position}");
                Console.WriteLine($"\tcg[{i}] = {kin[i].CgPosition}");
                Console.WriteLine($"\ts[{i}] = {kin[i].JointAxis}");
                Console.WriteLine($"\tv[{i}] = {kin[i].Velocity}");
                Console.WriteLine($"\tκ[{i}] = {kin[i].BiasAcceleration}");
                Console.WriteLine($"\tw[{i}] = {kin[i].AppliedForce}");
                Console.WriteLine($"\tI[{i}] = {kin[i].SpatialInertia}");
                Console.WriteLine($"\tm[{i}] = {kin[i].Momentum}");
                Console.WriteLine($"\tp[{i}] = {kin[i].BiasForce}");
                Console.WriteLine($"\ta[{i}] = {kin[i].Acceleration}");
                Console.WriteLine($"\tam[{i}] = {kin[i].CgAccleration}");
                Console.WriteLine($"\tf[{i}] = {kin[i].Force}");
                Console.WriteLine($"\tsf[{i}] = {frc[i]}");
            }

        }

    }
}
