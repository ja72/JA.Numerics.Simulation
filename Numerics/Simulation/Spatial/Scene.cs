using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Scene : IHasUnits<Scene>, INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        public Scene(UnitSystem units = UnitSystem.MMKS)
        {
            Units = units;
            Time = float.NaN;
            Current = null;
            BodyList = new List<Solid>();
            GeometryList = new List<Mesh>();
            Gravity = Vector3.Zero;
        }

        public Scene(UnitSystem units, IEnumerable<Solid> bodies, WorldState current) : this(units)
        {
            Current = current;
        }

        Scene(UnitSystem units, Vector3 gravity, float time, Solid[] bodies, WorldState current) : this(units)
        {
            Gravity = gravity;
            Time = time;
            BodyList = new List<Solid>(bodies);
            Current = current;
        }
        [Category("Simulation")]
        public Vector3 Gravity { get; set; }
        [Category("Simulation")]
        public float Time { get; set; }
        [Browsable(false)]
        public List<Mesh> GeometryList { get; }
        [Category("Model")]
        public Mesh[] Geometries { get => GeometryList.ToArray(); }
        [Browsable(false)]
        public List<Solid> BodyList { get; }
        [Category("Model")]
        public Solid[] Bodies { get => BodyList.ToArray(); }
        [Category("Simulation")]
        public WorldState Current { get; set; }
        [Category("Model")]
        public UnitSystem Units { get; }
        public WorldState GetInitialConditions()
        {
            return new WorldState(Units, BodyList.Select((item) => item.State).ToArray());
        }

        public void Update(float elapsedTime)
        {
            if (float.IsNaN(Time))
            {
                Reset();
            }
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

        public WorldState CalcRate(float time, WorldState next)
        {
            var states = new BodyState[next.Count];
            for (int i = 0; i < states.Length; i++)
            {
                var rb = BodyList[i];
                var state = next.State[i];
                var pose = state.Pose;
                var cg = Pose.FromLocal(pose, rb.MassProperties.CG);
                var vel = rb.GetMotion(state, cg);
                var frc = BodyForce(rb, time, pose, vel);
                states[i] = BodyRate(rb, next.State[i], cg, vel, frc);
            }

            return new WorldState(Units, states);
        }
        public Vector33 BodyForce(Solid solid, float time, Pose pose, Vector33 velocity)
        {
            return solid.MassProperties.GetWeight(pose, Gravity);
        }

        public BodyState BodyRate(Solid sold, BodyState state, Vector3 cg, Vector33 motion, Vector33 force)
        {
            //tex: The rate of the state vector as summed at the origin
            //$$\begin{gathered}Y=\begin{pmatrix}r\\q\\p\\L\end{pmatrix} & \begin{gathered}\dot{r}=v\\
            //\dot{q}=\tfrac{1}{2}\omega\otimes q\\\dot{p}=F\\\dot{L}=\tau-v\times p\end{gathered}\end{gathered}$$

            var rp = Vector33.TwistValue(motion, state.Pose.Position);
            var qp = state.Pose.Orientation.Derivative(motion.Vector2);
            var pp = force.Vector1;
            var Lp = force.Vector2 - LinearAlgebra.Cross(rp, state.Momentum.Vector1);

            return new BodyState(
                state.Units,
                new Pose(rp, qp),
                new Vector33(pp, Lp));
        }

        public void Reset()
        {
            Time = 0;
            Current = GetInitialConditions();
            OnPropertyChanged(nameof(Current));
        }

        public Scene ConvertTo(UnitSystem target)
        {
            float fl = UnitFactors.Length(Units, target);
            return new Scene(target,
                fl*Gravity,
                Time,
                BodyList.Select((s) => s.ConvertTo(target)).ToArray(),
                Current.ConvertTo(target));
        }
    }
}
