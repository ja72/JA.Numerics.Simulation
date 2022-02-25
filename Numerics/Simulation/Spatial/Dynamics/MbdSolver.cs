using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Numerics;
using JA.Numerics;
using JA.UI;

namespace JA.Numerics.Simulation.Spatial.Solvers
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class MbdSolver :
        INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        internal void OnPropertyChanged(string propertyName)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        public MbdSolver(World3 world)
        {            
            Bodies = world.BodyList.Select(rb => new Solid(rb) { Units = UnitSystem.SI } ).ToArray();
            Gravity = Unit.Acceleration.Convert(world.Units, UnitSystem.SI) * world.Gravity;

            Reset();
        }

        #region Properties
        [Category("Model")]
        public UnitSystem Units { get; }
        [Category("Model")]
        public Solid[] Bodies { get; }
        [Category("Simulation")]
        public Vector3 Gravity { get; }
        [Category("Simulation")]
        public float Time { get; set; }
        [Category("Simulation")]
        public MbdStates Current { get; set; }
        #endregion

        #region Methods

        public void Reset()
        {
            Time = 0;
            Current = new MbdStates(Bodies.Select((item) => item.GetInitialState()).ToArray());
        }
        public void Update(float elapsedTime)
        {
            var next = Current;
            var K0 = CalcRate(Time, next);

            next = Current + elapsedTime / 2 * K0;
            var K1 = CalcRate(Time+elapsedTime/2, next);

            next = Current + elapsedTime / 2 * K1;
            var K2 = CalcRate(Time+elapsedTime/2, next);

            next = Current + elapsedTime * K2;
            var K3 = CalcRate(Time + elapsedTime, next);

            Time += elapsedTime;
            Current += elapsedTime / 6 * (K0 + 2 * K1 + 2 * K2 + K3);

            OnPropertyChanged(nameof(Time));
            OnPropertyChanged(nameof(Current));
        }

        public MbdStates CalcRate(float time, MbdStates next)
        {
            var states = new BodyState[next.Count];
            for (int i = 0; i < states.Length; i++)
            {
                var rb = Bodies[i];
                var state = next.State[i];
                var pose = Pose.FromLocal(state.Pose, rb.MeshOrigin);
                var cg = Pose.FromLocal(state.Pose, rb.CG);
                var vel = rb.GetMotion(state, cg);
                var frc = BodyForce(rb, time, pose, vel);
                states[i] = BodyRate(rb, next.State[i], cg, vel, frc);
            }

            return new MbdStates(states);
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

            var r = state.Pose.Position;
            var rp = Vector33.TwistValue(motion, r);
            var qp = state.Pose.Orientation.Derivative(motion.Vector2);
            var pp = force.Vector1;
            var Lp = (force.Vector2 - rp.Cross(state.Momentum.Vector1));

            return new BodyState(
                new Pose(rp, qp),
                new Vector33(pp, Lp));
        }

        public void Render(Graphics g, Camera camera, float scale = 1)
        {

            for (int k = 0; k < Bodies.Length; k++)
            {
                var body = Bodies[k];
                var state = Current.State[k];
                if (body.Mesh == null) continue;
                body.Render(g, camera, state.Pose, scale);
            }

        }

        #endregion

    }
}
