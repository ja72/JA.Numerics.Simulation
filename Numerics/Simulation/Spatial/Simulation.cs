using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using JA.Numerics.Simulation.Spatial;

namespace JA.Numerics.Simulation.Spatial
{

    public class Simulation
    {
        public Simulation(World3 world)
        {
            Gravity = world.Gravity;
            Objects = FromRootUp(world.Objects).ToArray();
            Parent = GetParents(Objects).ToArray();
            Children = GetChildren(Objects).ToArray();
            Count = Objects.Length;
            Time = 0;
            State = new JointState[Count];
            for (int i = 0; i < Count; i++)
            {
                State[i] = Objects[i].GetInitial();
            }
        }

        #region Static Helpers
        static IEnumerable<Link3> FromRootUp(IEnumerable<Link3> objects)
        {
            foreach (var level in objects.GroupBy(f => f.ParentIndex))
            {
                foreach (var item in level)
                {
                    yield return item;
                }
            }
        }
        static IEnumerable<int> GetParents(Link3[] list)
        {
            foreach (var item in list)
            {
                yield return Array.IndexOf(list, item.Parent);
            }
        }
        static IEnumerable<int[]> GetChildren(Link3[] list)
        {
            foreach (var item in list)
            {
                var children = list.Where(f => f.Parent == item);
                yield return children.Select(f => Array.IndexOf(list, f)).ToArray();
            }
        }
        #endregion

        public Vector3 Gravity { get; }
        public Link3[] Objects { get; }
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
        public JointState[] State { get; private set; }

        public JointState[] CalcDynamics(out FrameKinematics[] kin)
            => CalcDynamics(Time, State, out kin);
        public JointState[] CalcDynamics(float time, JointState[] state, out FrameKinematics[] kin)
        {
            int n = Objects.Length;
            kin = new FrameKinematics[n];
            for (int i = 0; i < n; i++)
            {
                var item = Objects[i];
                var i_parent = Parent[i];
                var prev = i_parent >= 0 ? kin[i_parent] : FrameKinematics.Zero;
                // go to frame, center of mass, or base of joint
                var pose = prev.Position * item.LocalPosition;
                var cg = Pose.FromLocal(pose, item.MassProperties.CG);
                var vel = prev.Velocity;
                var acc = prev.Acceleration;
                // joint properties
                pose *= item.GetLocalStep(state[i].Angle);
                var si = item.GetAxis(pose);
                vel += si * state[i].Speed;
                var kap = Vector33.CrossTwistTwist( vel,  (si * state[i].Speed));
                acc += si * state[i].Aceleration + kap;
                // mass properties
                var I = item.MassProperties.Spi(pose.Orientation, cg);
                var M = item.MassProperties.Spm(pose.Orientation, cg);
                var wt = item.MassProperties.GetWeight(Gravity, cg);
                if (item.AppliedForce != null)
                {
                    wt += item.AppliedForce(time, pose, vel);
                }
                var mom = I * vel;
                var pee = Vector33.CrossTwistWrench(vel, mom);
                var frc = I * acc + pee;
                frc += wt;
                kin[i] = new FrameKinematics(
                    pose,
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
                var RUi = 1 - Vector33.Outer(Ti, si);

                art[i] = new FrameArticulated(I, d, Ti, RUi);
            }
            JointState[] dyn = new JointState[n];
            for (int i = 0; i < n; i++)
            {
                Link3 item = Objects[i];
                int i_parent = Parent[i];
                FrameKinematics prev = i_parent >= 0 ? kin[i_parent] : FrameKinematics.Zero;
                float q = state[i].Angle;
                float qp = state[i].Speed;
                float qpp = state[i].Aceleration;
                float tau = state[i].Torque;

                Vector33 si = kin[i].JointAxis;
                Matrix33 Ai = art[i].ArticulatedInertia;
                Vector33 di = art[i].ArticulatedForces;
                Vector33 Ti = art[i].PrecussionAxis;
                Matrix33 RUi = art[i].ReactionSpace;
                Vector33 κi = kin[i].BiasAcceleration;
                Vector33 a0 = prev.Acceleration;
                Vector33 ai;
                Vector33 fi;
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
                dyn[i] = new JointState(q, qp, qpp, tau);
            }

            return dyn;
        }

        public Vector33[] CheckForceBalance(FrameKinematics[] kin)
        {
            int n = kin.Length;
            Vector33[] result = new Vector33[n];
            for (int index = n - 1; index >= 0; index--)
            {
                Vector33 Fn = Vector33.Zero;
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

    }
}
