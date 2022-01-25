﻿using System.Collections.Generic;
using System.ComponentModel;
using System.Numerics;
using static System.Windows.Forms.AxHost;

namespace JA.Numerics.Simulation.Spatial
{

    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class Solid : Body, IHasUnits<Solid>
    {
        public Solid(Mesh mesh, World3 scene, float mass, Pose meshLocation)
            : this(mesh, scene, mesh.GetMmoiFromMass(mass), meshLocation)
        { }
        public Solid(Mesh mesh, World3 scene, MassProperties mass, Pose meshLocation)
            : base(mesh, scene, mass, meshLocation)
        {
            InitialDisplacement = Pose.Origin;
            InitialTranslationalVelocity = Vector3.Zero;
            InitialRotationalVelocity = Vector3.Zero;
        }
        public Solid(Solid copy, UnitSystem target) : base(copy, target)
        {
            float fl = UnitFactors.Length(copy.Units, target);
            InitialDisplacement                    = copy.InitialDisplacement.ConvertFromTo(copy.Units, target);
            InitialTranslationalVelocity           = fl* copy.InitialTranslationalVelocity;
            InitialRotationalVelocity              = copy.InitialRotationalVelocity;
        }
        public Solid(Solid copy) : base(copy)
        {
            InitialDisplacement                    = copy.InitialDisplacement;
            InitialTranslationalVelocity           = copy.InitialTranslationalVelocity;
            InitialRotationalVelocity              = copy.InitialRotationalVelocity;
        }

        public Pose InitialDisplacement { get; set; }
        public Vector3 InitialTranslationalVelocity {get; set; }
        public Vector3 InitialRotationalVelocity { get; set; }

        public BodyState GetInitialState()
        {
            var cg = Pose.FromLocal(InitialDisplacement, CG);
            var motion = GetMotion(InitialTranslationalVelocity, InitialRotationalVelocity, cg);
            return GetState(InitialDisplacement, cg, motion);
        }

        public new Solid ConvertTo(UnitSystem target)
        {
            return new Solid(this, target);
        }
        public BodyState GetState(Pose pose, Vector3 cg, Vector33 motion)
        {
            var I = MassProperties.Spi(pose.Orientation, cg);
            //tex: Momentum from velocity
            //$$\begin{Bmatrix}p\\L\end{Bmatrix} = \begin{bmatrix}m & -m\,c\times\\
            //m\,c\times & I_{C}-m\,c\times c\times\end{bmatrix}\begin{Bmatrix}v\\\omega\end{Bmatrix}$$

            var L = I*motion;
            return new BodyState(
                pose,
                L);
        }
        public BodyState GetState(BodyState state, Vector33 motion)
        {
            var cg = Pose.FromLocal(state.Pose, CG);
            return GetState(state.Pose, cg, motion);
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
            var cg = Pose.FromLocal(state.Pose, CG);
            return GetMotion(state, cg);
        }
        public Vector33 GetMotion(Vector3 vee, Vector3 omg, Pose pivot)
        {
            vee = vee.Rotate(pivot.Orientation);
            omg = omg.Rotate(pivot.Orientation);
            return Vector33.Twist(vee, omg, pivot.Position);
        }
        public void SetMotion(Vector3 vee, Vector3 omg)
        {
            InitialTranslationalVelocity  = vee;
            InitialRotationalVelocity = omg;
        }
        public void SetMotion(Vector33 motion)
{
            var cg = Pose.FromLocal(InitialDisplacement, CG);
            InitialTranslationalVelocity = motion.TwistAt(cg);
            InitialRotationalVelocity = motion.Vector2;
        }
        public override string ToString()
        {
            return $"Solid(Mass={MassProperties.Mass}, CG={CG})";
        }
    }
}
