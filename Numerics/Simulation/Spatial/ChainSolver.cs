using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Numerics;
using JA.Numerics.Simulation.Spatial;
using JA.Numerics.UI;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TaskbarClock;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class ChainSolver : 
        IVisible,
        IHasUnits<ChainSolver>,
        INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        public ChainSolver(World3 world)
        {
            World = world;
            Units = world.Units;
            Gravity = world.Gravity;
            Bodies = FromRootUp(world.Links).ToArray();
            Parent = GetParents(Bodies).ToArray();
            Children = GetChildren(Bodies).ToArray();

            Reset();
        }

        ChainSolver(World3 world, Vector3 gravity, Link3[] chain, int[] parent, int[][] children, float time, ChainStates current, UnitSystem target) : this(world)
        {
            float fl = UnitFactors.Length(world.Units, target);
            Units = target;
            Gravity=fl*gravity;
            Bodies=chain.Select((item)=>item.ConvertTo(target)).ToArray();
            Parent=parent;
            Children=children;
            Time = time;
            Current=current;
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

        #region Properties
        [Category("Model")]
        public UnitSystem Units { get; }
        [Category("Model")]
        public World3 World { get; }
        [Category("Simulation")]
        public Vector3 Gravity { get; }
        [Category("Model")]
        public Link3[] Bodies { get; }
        [Browsable(false)]
        public int Count { get => Bodies.Length; }
        /// <summary>
        /// Index of parent for each object.
        /// </summary>        
        [Browsable(false)]
        [Category("Model")]
        public int[] Parent { get; }
        /// <summary>
        /// Index of children for each frame.
        /// </summary>
        /// <value>The children.</value>
        [Browsable(false)]
        [Category("Model")]
        public int[][] Children { get; }
        [Category("Simulation")]
        public float Time { get; set; }
        [Category("Simulation")]
        public ChainStates Current { get; private set; } 
        #endregion

        public FrameKinematics[] GetKinematics(float time, ChainStates states) 
        {
            int n = Bodies.Length;
            var kin = new FrameKinematics[n];
            for (int i = 0; i < n; i++)
            {
                var item = Bodies[i];
                var state = states[i];
                var i_parent = Parent[i];
                var prev = i_parent >= 0 ? kin[i_parent] : FrameKinematics.Zero;
                // go to frame, center of mass, or base of joint
                var jpose = Pose.FromLocal(prev.Position, item.LocationOnParent);
                var mpose = Pose.FromLocal(jpose, item.MeshOrigin);
                var cg = Pose.FromLocal(mpose, item.MassProperties.CG);
                var vel = prev.Velocity;
                var acc = prev.Acceleration;
                // joint properties
                jpose = Pose.FromLocal(jpose, item.GetLocalStep(state.Angle));
                var si = item.GetAxis(jpose);
                vel += si * state.Speed;
                var kap = Vector33.CrossTwistTwist(vel, (si * state.Speed));
                // mass properties
                var I = item.MassProperties.Spi(mpose.Orientation, cg);
                var M = item.MassProperties.Spm(mpose.Orientation, cg);
                var wt = item.MassProperties.GetWeight(Gravity, cg);
                if (item.AppliedForce != null && item.AppliedForce != World3.ZeroForce)
                {
                    wt += item.AppliedForce(time, mpose, vel);
                }
                var info = new JointInfo(item.JointProperties, time, state);
                var mom = I * vel;
                var pee = Vector33.CrossTwistWrench(vel, mom);
                var frc = I * acc + pee;
                frc += wt;
                kin[i] = new FrameKinematics(
                    info,
                    jpose,
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
            return kin;
        }

        public FrameArticulated[] GetArticulated(FrameKinematics[] kin)
        {
            int n = Bodies.Length;
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
                    var Qn = kin[i_child].Joint.Torque;
                    I += RUn * An;
                    d += RUn * (An * κn + dn) + Tn * Qn;
                }
                var si = kin[i].JointAxis;
                var Ti = I * si / (si * (I * si));
                var RUi = 1 - Vector33.Outer(Ti, si);

                art[i] = new FrameArticulated(I, d, Ti, RUi);
            }
            return art;
        }

        public JointInfo[] CalcDynamics(out FrameKinematics[] kin)
            => CalcDynamics(Time, Current, out kin);
        public JointInfo[] CalcDynamics(float time, ChainStates state, out FrameKinematics[] kin)
        {
            int n = Bodies.Length;
            kin = GetKinematics(time, state);
            var art = GetArticulated(kin);

            JointInfo[] dyn = new JointInfo[n];
            for (int i = 0; i < n; i++)
            {
                Link3 item = Bodies[i];
                int i_parent = Parent[i];
                FrameKinematics prev = i_parent >= 0 ? kin[i_parent] : FrameKinematics.Zero;
                float q = state[i].Angle;
                float qp = state[i].Speed;
                float qpp = kin[i].Joint.Acceleration;
                float tau = kin[i].Joint.Torque;

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
                dyn[i] = new JointInfo(q, qp, qpp, tau, item.JointProperties);
            }

            return dyn;
        }
        public ChainStates CalcRate(float time, ChainStates next)
        {
            var dyn = CalcDynamics(time, next, out _);
            return new ChainStates(dyn.Select((item) => item.GetRate()).ToArray());
        }

        public void Reset()
        {
            Time = 0;
            Current = new ChainStates(Bodies.Select((item) => item.GetInitial()).ToArray());
        }

        public void Update(float elapsedTime)
        {
            var next = Current;
            var K0 = CalcRate(Time, next);

            next = Current + (elapsedTime / 2) * K0;
            var K1 = CalcRate(Time+elapsedTime/2, next);

            next = Current + (elapsedTime / 2) * K1;
            var K2 = CalcRate(Time+elapsedTime/2, next);

            next = Current + elapsedTime * K2;
            var K3 = CalcRate(Time + elapsedTime, next);

            Time += elapsedTime;
            Current += (elapsedTime / 6) * (K0 + 2 * K1 + 2 * K2 + K3);

            OnPropertyChanged(nameof(Time));
            OnPropertyChanged(nameof(Current));
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

        public void Render(Graphics g, Camera camera)
        {
            var kin = GetKinematics(Time, Current);
            for (int k = 0; k < Bodies.Length; k++)
            {
                var body = Bodies[k];
                var state = Current[k];
                if (body.Mesh == null) continue;
                var mesh = body.Mesh;
                var pose = Pose.FromLocal(kin[k].Position, body.MeshOrigin);
                mesh.Render(g, camera, pose);
            }
        }

        public ChainSolver ConvertTo(UnitSystem target)
        {
            return new ChainSolver(World, Gravity, Bodies, Parent, Children, Time, Current, target);
        }
    }
}
