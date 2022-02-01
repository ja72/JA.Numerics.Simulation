using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Numerics;
using System.Windows.Forms;
using JA.Numerics;
using JA.UI;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.TaskbarClock;

namespace JA.Numerics.Simulation.Spatial.Solvers
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class ChainSolver :
        IHasUnits<ChainSolver>,
        INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        public ChainSolver(Chain chain)
        {
            World = chain.World;
            Units = chain.Units;
            Gravity = chain.World.Gravity;
            Bodies = FromRootUp(chain.linkList).ToArray();
            Parent = GetParents(Bodies).ToArray();
            Children = GetChildren(Bodies).ToArray();
            ContactPoint = chain.ContactPoint;
            ContactNormal = chain.ContactNormal;
            EnableContacts = false;
            Reset();
        }

        ChainSolver(ChainSolver copy, UnitSystem target)
        {
            float fl = UnitFactors.Length(copy.Units, target);
            World = copy.World;
            Units = target;
            Gravity=fl*copy.Gravity;
            Bodies=copy.Bodies.Select((item) => item.ConvertTo(target)).ToArray();
            Parent=copy.Parent;
            Children=copy.Children;
            Time = copy.Time;
            Current=copy.Current.ConvertFromTo(copy.Units, target);
            ContactNormal = copy.ContactNormal;
            ContactPoint = fl * copy.ContactPoint;
            EnableContacts = copy.EnableContacts;
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

        /// <summary>
        /// Direction of Contact
        /// </summary>
        [Category("Simulation")] 
        public Vector3 ContactNormal { get; set; }
        /// <summary>
        /// Location of Contact
        /// </summary>
        [Category("Simulation")] 
        public Vector3 ContactPoint { get; set; }
        /// <summary>
        /// Gets or sets a value indicating whether contacts are considered.
        /// </summary>
        [Category("Simulation")]
        public bool EnableContacts { get; set; }
        #endregion

        public FrameKinematics[] GetKinematics(float time, ChainStates states)
        {
            int n = Bodies.Length;
            var kin = new FrameKinematics[n];
            for (int i = 0; i < n; i++)
            {
                var body = Bodies[i];
                var state = states[i];
                var i_parent = Parent[i];
                var prev = i_parent >= 0 ? kin[i_parent] : FrameKinematics.Zero;
                // go to frame, center of mass, or base of joint
                // bottom of joint
                var bot = Pose.FromLocal(prev.Position, body.LocationOnParent);
                // top of joint
                var top = Pose.FromLocal(bot, body.JointProperties.GetLocalStep(state.Angle));
                // mesh origin
                var mpose = Pose.FromLocal(top, body.MeshOrigin);
                // center of mass
                var cg = Pose.FromLocal(mpose, body.MassProperties.CG);
                // joint properties
                var Joint = new JointInfo(body.JointProperties, time, state);
                var s = body.JointProperties.GetAxis(top);
                var Δv = s * state.Speed;
                var v = prev.Velocity + Δv;
                var κ = Vector33.CrossTwistTwist(v, Δv);
                var a = prev.Acceleration + s*Joint.Acceleration + κ;
                // mass properties
                var I = body.MassProperties.Spi(mpose.Orientation, cg);
                var M = body.MassProperties.Spm(mpose.Orientation, cg);
                var w = body.MassProperties.GetWeight(Gravity, cg);
                if (body.AppliedForce != null && body.AppliedForce != World3.ZeroForce)
                {
                    w += body.AppliedForce(time, mpose, v);
                }
                var ell = I * v;
                var p = Vector33.CrossTwistWrench(v, ell);
                var f = I * a + p - w;

                kin[i] = new FrameKinematics(
                    Joint, top, cg, s, v, κ, 
                    I, M,
                    ell, p, w, a, f);
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
                var Ti = I * si / Vector33.Dot(si, I * si);
                var RUi = 1 - Vector33.Outer(Ti, si);

                art[i] = new FrameArticulated(I, d, Ti, RUi);
            }
            return art;
        }

        public JointInfo[] CalcDynamics(out FrameKinematics[] kin, out FrameArticulated[] art)
            => CalcDynamics(Time, Current, out kin, out art);
        public JointInfo[] CalcDynamics(float time, ChainStates state, out FrameKinematics[] kin, out FrameArticulated[] art)
        {
            int n = Bodies.Length;
            kin = GetKinematics(time, state);
            art = GetArticulated(kin);

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
                        tau = Vector33.Dot(si, (Ai * (si * qpp + a0 + κi) + di));
                        fi = Ti * tau + RUi * (Ai * (a0 + κi) + di);
                        break;
                    case Prescribed.Load:
                        // qpp = (Q - sᵀ*(A*(κ + a0) + d))/(sᵀ*A*s)
                        qpp = (tau - Vector33.Dot(si, (Ai * (a0 + κi) + di))) / Vector33.Dot(si, (Ai * si));
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

        public int[] GetConstrainedInverseInertia(int index, FrameKinematics[] kin, FrameArticulated[] art, out Matrix33[] Φ, out Matrix33[] Y)
        {
            // Assume an impulse is applied to the indexed body
            // Find the chain of bodies to the ground
            if (index<0)
            {
                Φ=null;
                Y=null;
                return Array.Empty<int>();
            }

            var chain = new List<int>();
            do
            {
                chain.Add(index);
                index = Parent[index];
            } while (index>=0);
            chain.Reverse();
            int n = chain.Count;
            Φ = new Matrix33[n];
            var Λ = new Matrix33[n];
            Y = new Matrix33[n];
            var Φn = Matrix33.Identity;
            for (int i = n - 1; i >= 0; i--)
            { 
                index = chain[i]; 
                var si = kin[index].JointAxis;
                var IA = art[index].ArticulatedInertia;
                Λ[index] = Vector33.Outer(si, si)/Vector33.Dot(si, IA*si);
                Φ[index] = (1-IA*Λ[index])*Φn;
                Φn = Φ[index];
            }
            var Yp = Matrix33.Zero;
            for (int i = 0; i < n; i++)
            {
                index = chain[i];
                var IA = art[index].ArticulatedInertia;
                Φn = i<n-1 ? Φ[chain[i+1]] : Matrix33.Identity;
                Y[index] = (1-Λ[index]*IA)*Yp + Λ[index]*Φn;
                Yp = Y[index];
            }

            return chain.ToArray();
        }

        public bool HandleContact(float time, ref ChainStates states)
        {
            if(ContactNormal.LengthSquared()==0) return false;
            var kin = GetKinematics(time, states);
            var art = GetArticulated(kin);
            int index = Bodies.Length-1;
            var pos = Pose.FromLocal(kin[index].Position, Bodies[index].LocalMarker);
            var d = Vector3.Dot(ContactNormal, pos - ContactPoint);
            var n = Vector33.Wrench(ContactNormal, ContactPoint, 0);
            var chain = GetConstrainedInverseInertia(index, kin, art, out var Φ, out var Y);
            var v_imp = Vector33.Dot(n, kin[index].Velocity);
            if (d<=0 && v_imp<0)
            {
                var m_imp = 1/Vector33.Dot(n, Y[index]*n);
                var J = (1+0f)*m_imp*v_imp;
                var Yp = Matrix33.Zero;
                for (int i = 0; i < chain.Length; i++)
                {
                    index = chain[i];
                    var state = states.State[index];
                    var si = kin[index].JointAxis;
                    var IA = art[index].ArticulatedInertia;
                    var Φn = i<chain.Length-1 ? Φ[chain[i+1]] : Matrix33.Identity;
                    var Δqp = -Vector33.Dot(si, (Φn-IA*Yp)*n)/Vector33.Dot(si, IA*si)*J;
                    states.State[index] = new JointState(state.Type, state.Angle, state.Speed +  Δqp);
                    Yp = Y[index];
                }

                return true;
            }
            return false;
        }

        public ChainStates CalcRate(float time, ChainStates next)
        {
            var dyn = CalcDynamics(time, next, out _, out _);
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

            next = Current + elapsedTime / 2 * K0;
            if(EnableContacts) HandleContact(Time+elapsedTime/2, ref next);
            var K1 = CalcRate(Time+elapsedTime/2, next);

            next = Current + elapsedTime / 2 * K1;
            if (EnableContacts) HandleContact(Time+elapsedTime/2, ref next);
            var K2 = CalcRate(Time+elapsedTime/2, next);

            next = Current + elapsedTime * K2;
            if (EnableContacts) HandleContact(Time+elapsedTime, ref next);
            var K3 = CalcRate(Time + elapsedTime, next);

            Time += elapsedTime;
            next = Current + elapsedTime / 6 * (K0 + 2 * K1 + 2 * K2 + K3);
            if (EnableContacts) HandleContact(Time, ref next);
            Current = next;

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
            if (EnableContacts && ContactNormal.LengthSquared()>0)
            {
                var pts = camera.Project(ContactPoint, ContactPoint + ContactNormal);
                g.FillEllipse(Brushes.MediumPurple, pts[0].X-2, pts[0].Y-2, 4, 4);
                float scale = 1;
                using var pen = new Pen(Color.MediumPurple, 0);
                pen.CustomEndCap = new AdjustableArrowCap(2f*scale, 8f*scale, true);
                g.DrawLines(pen, pts);
            }

            var kin = GetKinematics(Time, Current);
            for (int k = 0; k < Bodies.Length; k++)
            {
                var body = Bodies[k];
                var pose = kin[k].Position;
                body.Render(g, camera, pose);
            }
        }

        public ChainSolver ConvertTo(UnitSystem target)
        {
            return new ChainSolver(this, target);
        }
    }
}
