using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace JA.Numerics.Simulation.Planar
{
    public class World2 
    {
        public World2() : this(Vector2.Zero) { }
        public World2(Vector2 gravity) 
        {
            this.objects = new List<Frame2>();
            this.Gravity = gravity;
        }
        internal List<Frame2> objects;
        public Vector2 Gravity { get; }
        public int Count { get => objects.Count; }
        public IReadOnlyList<Frame2> Objects { get => objects.AsReadOnly(); }
        internal void AddObject(Frame2 frame)
        {
            objects.Add(frame);
        }
        public Frame2 AddChild(Pose localPosition, MassProperties mass, JointProperties type, float initialDisplacement = 0, float initialSpeed = 0)
            => new Frame2(this, localPosition, mass, type, initialDisplacement, initialSpeed);
        public Frame2 SlideAlongX(Pose localPosition, MassProperties mass, float initialDisplacement = 0)
            => AddChild(localPosition, mass, JointType.SlideAlongX, initialDisplacement);
        public Frame2 SlideAlongY(Pose localPosition, MassProperties mass, float initialDisplacement = 0)
            => AddChild(localPosition, mass, JointType.SlideAlongY, initialDisplacement);
        public Frame2 AddRevolute(Pose localPosition, MassProperties mass, float initialAngle = 0)
            => AddChild(localPosition, mass, JointType.Revolute, initialAngle);
        public Frame2 AddCollarAlongX(Pose localPosition, MassProperties mass, float initialDisplacement =0, float initialAngle = 0)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongX, initialDisplacement)
                    .AddChild(Pose.Origin, mass, JointType.Revolute, initialAngle);
        public Frame2 AddCollarAlongY(Pose localPosition, MassProperties mass, float initialDisplacement = 0, float initialAngle = 0)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongY, initialDisplacement)
                    .AddChild(Pose.Origin, mass, JointType.Revolute, initialAngle);
        public Frame2 AddFree(Pose localPosition, MassProperties mass, float initialX, float intialY, float initialAngle)
            => AddChild(localPosition, MassProperties.Zero, JointType.SlideAlongX, initialX)
                    .AddChild(Pose.Origin, MassProperties.Zero, JointType.SlideAlongY, intialY)
                    .AddChild(Pose.Origin, mass, JointType.Revolute, initialAngle);

        public Frame2 AddChain(int count, Vector2 localPosition, MassProperties bodyProperty)
        {
            if (count == 0) return null;
            var joint = new JointProperties(JointType.Revolute);
            var item = AddChild(Vector2.Zero, bodyProperty, joint);
            for (int i = 1; i < count; i++)
            {
                item = item.AddChild(localPosition, bodyProperty, joint);
            }
            return item;
        }

        public void RemoveChild(Frame2 child)
        {
            foreach (var item in objects.Where(f => f.Parent == child))
            {
                item.Parent = null;
            }
            objects.Remove(child);
        }

        #region Formatting
        public override string ToString()
            => $"World(Count={Count}, Gravity={Gravity})";
        #endregion

        public Simulation Simulation() => new Simulation(this);

        public static void Demo()
        {
            Console.WriteLine("=== Planar Dynamics Demo ===");
            float m = 0.15f, L = 150f, h = 650f;
            var localPos = Vector2.UnitX * L;
            var body = MassProperties.Rectangle(localPos / 2, m, L, h);
            Console.WriteLine($"{body}");
            var world = new World2(-10 * Vector2.UnitY);
            world.AddChain(6, localPos, body);
            Console.WriteLine($"{world}");

            var sim = world.Simulation();

            var next = sim.CalcDynamics(out var kin);

            var x = kin.Select((k) => k.Position.Position.X).ToArray();
            var cg = kin.Select((k) => k.CgPosition.X).ToArray();
            var vcy = kin.Select((k) => k.Velocity.TwistAt(k.CgPosition).Y).ToArray();
            var acy = kin.Select((k) => k.CgAccleration.Y).ToArray();

            var qpp = next.Select((st) => st.Aceleration).ToArray();

            Console.WriteLine($"x = {string.Join(",", x)}");
            Console.WriteLine($"cg = {string.Join(",", cg)}");
            Console.WriteLine($"vcy = {string.Join(",", vcy)}");
            Console.WriteLine($"acy = {string.Join(",", acy)}");
            Console.WriteLine($"qpp = {string.Join(",", qpp)}");

            var frc = sim.CheckForceBalance(kin);

            for (int i = 0; i < sim.Objects.Length; i++)
            {
                Console.WriteLine($"{sim.Objects[i]}");
                Console.WriteLine($"\tq({i}) = {next[i].Angle}");
                Console.WriteLine($"\tqp({i}) = {next[i].Speed}");
                Console.WriteLine($"\tqpp({i}) = {next[i].Aceleration}");
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
