using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace JA.Numerics.Simulation.Planar
{

    public class Simulation
    {
        public Simulation(World2 world)
        {
            Gravity = world.Gravity;
            Objects = FromRootUp(world.Objects).ToArray();
            Parent = GetParents(Objects).ToArray();
            Children = GetChildren(Objects).ToArray();
            Count = Objects.Length;
            Time = 0;
            State = new FrameState[Count];
            for (int i = 0; i < Count; i++)
            {
                State[i] = Objects[i].GetInitial();
            }
        }

        #region Static Helpers
        static IEnumerable<Frame2> FromRootUp(IEnumerable<Frame2> objects)
        {
            foreach (var level in objects.GroupBy(f => f.ParentIndex))
            {
                foreach (var item in level)
                {
                    yield return item;
                }
            }
        }
        static IEnumerable<int> GetParents(Frame2[] list)
        {
            foreach (var item in list)
            {
                yield return Array.IndexOf(list, item.Parent);
            }
        }
        static IEnumerable<int[]> GetChildren(Frame2[] list)
        {
            foreach (var item in list)
            {
                var children = list.Where(f => f.Parent == item);
                yield return children.Select(f => Array.IndexOf(list, f)).ToArray();
            }
        }
        #endregion

        public Vector2 Gravity { get; }
        public Frame2[] Objects { get; }
        public int Count { get; }
        /// <summary>
        /// Index of parent for each object.
        /// </summary>
        public int[] Parent { get; }
        /// <summary>
        /// Index of children for each frame.
        /// </summary>
        /// <value>The children.</value>
        public int[][] Children { get; }
        public float Time { get; private set; }
        public FrameState[] State { get; private set; }

        public FrameState[] CalcDynamics(out FrameKinematics[] kin)
            => CalcDynamics(Time, State, out kin);
        public FrameState[] CalcDynamics(float time, FrameState[] state, out FrameKinematics[] kin)
        {
            int n = Objects.Length;
            kin = new FrameKinematics[n];
            for (int i = 0; i < n; i++)
            {
                var item = Objects[i];
                var i_parent = Parent[i];
                var prev = i_parent >= 0 ? kin[i_parent] : FrameKinematics.Zero;
                // go to frame, center of mass, or base of joint
                var pos = prev.Position + item.LocalPosition;
                var cg = (pos + item.MassProperties.CG).Position;
                var vel = prev.Velocity;
                var acc = prev.Acceleration;
                // joint properties
                pos += item.GetLocalStep(state[i].Angle);
                var si = item.GetAxis(pos);
                vel += si * state[i].Speed;
                var kap = Vector21.CrossTwistTwist( vel,  (si * state[i].Speed));
                acc += si * state[i].Aceleration + kap;
                // mass properties
                var I = Spi(item.MassProperties, pos);
                var M = Spm(item.MassProperties, pos);
                var wt = GetWeight(item.MassProperties, Gravity, cg);
                if (item.AppliedForce != null)
                {
                    wt += item.AppliedForce(time, pos, vel);
                }
                var mom = I * vel;
                var pee = Vector21.CrossTwistWrench(vel, mom);
                var frc = I * acc + pee;
                frc += wt;
                kin[i] = new FrameKinematics(
                    pos,
                    cg,
                    si,
                    vel,
                    kap,
                    I,
                    M,
                    mom,
                    pee,
                    wt,
                    acc,
                    frc);
            }
            for (int i = n - 1; i >= 0; i--)
            {
                var frc = kin[i].Force;
                foreach (var i_child in Children[i])
                {
                    frc += kin[i_child].Force;
                }
                kin[i].Force = frc;
            }
            var art = new FrameArticulated[n];
            for (int i = n - 1; i >= 0; i--)
            {
                var I = kin[i].SpatialInertia;
                var d = kin[i].BiasForce - kin[i].AppliedForce;
                foreach (var i_child in Children[i])
                {
                    var An = art[i_child].ArticulatedInertia;
                    var RUn = art[i_child].ReactionSpace;
                    var κn = kin[i_child].BiasAcceleration;
                    var dn = art[i_child].ArticulatedForces;
                    var Tn = art[i_child].PrecussionAxis;
                    var Qn = state[i_child].Torque;
                    I += RUn * An;
                    d += RUn * (An * κn + dn) + Tn * Qn;
                }
                var si = kin[i].JointAxis;
                var Ti = I * si / (si * (I * si));
                var RUi = 1 - Vector21.Outer(Ti, si);

                art[i] = new FrameArticulated(I, d, Ti, RUi);
            }
            FrameState[] dyn = new FrameState[n];
            for (int i = 0; i < n; i++)
            {
                Frame2 item = Objects[i];
                int i_parent = Parent[i];
                FrameKinematics prev = i_parent >= 0 ? kin[i_parent] : FrameKinematics.Zero;
                float q = state[i].Angle;
                float qp = state[i].Speed;
                float qpp = state[i].Aceleration;
                float tau = state[i].Torque;

                Vector21 si = kin[i].JointAxis;
                Matrix21 Ai = art[i].ArticulatedInertia;
                Vector21 di = art[i].ArticulatedForces;
                Vector21 Ti = art[i].PrecussionAxis;
                Matrix21 RUi = art[i].ReactionSpace;
                Vector21 κi = kin[i].BiasAcceleration;
                Vector21 a0 = prev.Acceleration;
                Vector21 ai;
                Vector21 fi;
                switch (item.JointProperties.Motion)
                {
                    case Prescribed.Motion:
                        // Q = sᵀ*(A*(s*qpp + κ + a0) + d)
                        ai = a0 + si * qpp + κi;
                        tau = si * (Ai * (si * qpp + a0 + κi) + di);
                        fi = Ti * tau + RUi * (Ai * (a0 + κi) + di);
                        break;
                    case Prescribed.Load:
                        // qpp = (Q - sᵀ*(A*(κ + a0) + d))/(sᵀ*A*s)
                        qpp = (tau - si * (Ai * (a0 + κi) + di)) / (si * (Ai * si));
                        ai = a0 + si * qpp + κi;
                        fi = Ai * ai + di;
                        break;
                    default:
                        throw new NotSupportedException($"{item.JointProperties.Motion} is not supported.");
                }                
                kin[i].Acceleration = ai;
                kin[i].Force = fi;
                dyn[i] = new FrameState(q, qp, qpp, tau);
            }

            return dyn;
        }

        public Vector21[] CheckForceBalance(FrameKinematics[] kin)
        {
            int n = kin.Length;
            Vector21[] result = new Vector21[n];
            for (int index = n - 1; index >= 0; index--)
            {
                Vector21 Fn = Vector21.Zero;
                foreach (int i_child in Children[index])
                {
                    Fn += kin[i_child].Force;
                }

                var Fi = kin[index].Force;
                var Wi = kin[index].AppliedForce;
                var Gi = kin[index].SpatialInertia * kin[index].Acceleration + kin[index].BiasForce;

                result[index] = Fi + Wi - Fn - Gi;
            }
            return result;
        }
        public static Vector2 MaterialAcceleration(
            Vector2 spatialAcceleration,
            Vector2 velocity,
            float omega)
        => spatialAcceleration + LinearAlgebra.Cross(omega, velocity);

        public static Vector2 SpartialAcceleration(
            Vector2 materialAcceleration,
            Vector2 velocity,
            float omega)
        => materialAcceleration - LinearAlgebra.Cross(omega, velocity);
        public static Vector21 GetWeight(MassProperties body, Vector2 gravity, Vector2 cg) 
            => Vector21.Wrench(body.data.mass * gravity, cg);

        public static Matrix21 Spi(float mass, float mmoi, Vector2 cg)
        {
            return Matrix21.Symmetric(
                Matrix2.Diagonal(mass), -mass * LinearAlgebra.Cross(cg, 1),
                mmoi + mass * cg.LengthSquared());
        }
        public static Matrix21 Spi(MassProperties body, Pose pose)
        {
            var cg = (pose + body.data.cg).Position;
            return Spi(body.data.mass, body.data.mmoi, cg);
        }
        public static Matrix21 Spm(float mass, float mmoi, Vector2 cg)
        {
            return Matrix21.Symmetric(
                1 / mass + LinearAlgebra.Mmoi(cg) / mmoi,
                LinearAlgebra.Cross(cg, 1) / mmoi,
                1 / mmoi);
        }

        public static Matrix21 Spm(MassProperties body, Pose pose)
        {
            var cg = (pose + body.data.cg).Position;
            return Spm(body.data.mass, body.data.mmoi, cg);
        }

    }
}
