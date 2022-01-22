using System;
using System.Collections.Generic;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial
{

    public class Solid :
        IHasUnits<Solid>
    {
        public Solid(Scene scene, float mass, Vector3 mMoi, Vector3 cg, Pose pose)
        {
            Units = scene.Units;
            Scene = scene;
            var I = Matrix3.Diagonal(mMoi);
            MassProperties = new MassProperties(scene.Units, mass, I, cg);
            State=new BodyState(scene.Units, pose, Vector33.Zero);
            Mesh = null;
        }
        public Solid(Scene scene, float mass, Mesh mesh, Pose pose)
        {
            Units = scene.Units;
            Scene = scene;
            MassProperties = mesh.GetMmoiFromMass(mass);
            State=new BodyState(scene.Units, pose, Vector33.Zero);
            Mesh = mesh;
        }
        public Solid(UnitSystem units, Scene scene, Mesh mesh, Pose pose, MassProperties massProperties, BodyState state)
        {
            Units = units;
            Scene = scene;
            Mesh = mesh;
            LocalPosition = pose;
            MassProperties = massProperties;
            State = state;
        }
        public UnitSystem Units { get; }
        public Scene Scene { get; }
        public Mesh Mesh { get; }
        public Pose LocalPosition { get; }
        public MassProperties MassProperties { get; private set; }
        public BodyState State { get; set; }

        public Solid ConvertTo(UnitSystem target)
        {
            return new Solid(
                target,
                Scene, Mesh.ConvertTo(target),
                LocalPosition.ConvertFromTo(Units,target),
                MassProperties.ConvertTo(target),
                State.ConvertTo(target));
                
        }
        public BodyState SetMotion(BodyState state, Vector3 cg, Vector33 motion)
        {
            var I = MassProperties.Spi(state.Pose.Orientation, cg);
            //tex: Momentum from velocity
            //$$\begin{Bmatrix}p\\L\end{Bmatrix} = \begin{bmatrix}m & -m\,c\times\\
            //m\,c\times & I_{C}-m\,c\times c\times\end{bmatrix}\begin{Bmatrix}v\\\omega\end{Bmatrix}$$

            var L = I*motion;
            return new BodyState(
                state.Units,
                state.Pose,
                L);
        }
        public BodyState SetMotion(BodyState state, Vector33 motion)
        {
            var cg = Pose.FromLocal(state.Pose, MassProperties.CG);
            return SetMotion(state, cg, motion);
        }
        public void SetMotion(Vector33 motion)
        {
            State = SetMotion(State, motion);
        }
        public Vector33 GetMotion(BodyState state, Vector3 cg)
        {
            var M = MassProperties.Spm(state.Pose.Orientation, cg);
            //tex: Velocity from momentum
            // $$\begin{Bmatrix}v\\\omega\end{Bmatrix} =
            // \begin{bmatrix}\frac{1}{m}-c\times I_{C}^{-1}c\times & c\times I_{C}^{-1}\\
            //-I_{C}^{-1}c\times & I_{C}^{-1}\end{bmatrix}\begin{Bmatrix}p\\L\end{Bmatrix}$$

            var L = state.Momentum;
            return M*L;
        }
        public Vector33 GetMotion(BodyState state)
        {
            var cg = Pose.FromLocal(state.Pose, MassProperties.CG);
            return GetMotion(state, cg);
        }
        public Vector33 GetMotion() => GetMotion(State);

    }
}
