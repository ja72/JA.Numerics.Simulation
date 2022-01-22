using System.Numerics;

namespace JA.Numerics.Simulation.Planar
{
    public struct FrameKinematics
    {
        public FrameKinematics(
            in Pose position,
            in Vector2 cgPosition,
            in Vector21 jointAxis,
            in Vector21 velocity,
            in Vector21 biasAcceleration,
            in Matrix21 spatialInertia,
            in Matrix21 spatialMobility,
            in Vector21 momentum,
            in Vector21 biasForce,
            in Vector21 appliedForce,
            in Vector21 acceleration,
            in Vector21 force) : this()
        {
            Position = position;
            CgPosition = cgPosition;
            JointAxis = jointAxis;
            Velocity = velocity;
            BiasAcceleration = biasAcceleration;
            SpatialInertia = spatialInertia;
            SpatialMobility = spatialMobility;
            Momentum = momentum;
            BiasForce = biasForce;
            AppliedForce = appliedForce;
            Acceleration = acceleration;
            Force = force;
        }
        public Pose Position { get; }
        public Vector2 CgPosition { get; }
        public Vector21 JointAxis { get; }
        public Vector21 Velocity { get; }
        public Vector21 BiasAcceleration { get; }
        public Matrix21 SpatialInertia { get; }
        public Matrix21 SpatialMobility { get; }
        public Vector21 Momentum { get; }
        public Vector21 BiasForce { get; }
        public Vector21 AppliedForce { get; }
        public Vector21 Acceleration { get; set; }
        public Vector21 Force { get; set; }
        public Vector2 MaterialAccelerationAt(Vector2 target)
        {
            return Simulation.MaterialAcceleration( 
                Acceleration.TwistAt(target),
                Velocity.TwistAt(target), 
                Velocity.Scalar);
        }
        public Vector2 CgAccleration { get => MaterialAccelerationAt(CgPosition); }

        public Vector21 GetInertialForce()
        {
            return BiasForce + SpatialInertia * Acceleration;
        }

        public static readonly FrameKinematics Zero = new FrameKinematics(
            Pose.Origin,
            Vector2.Zero,
            Vector21.Zero,
            Vector21.Zero,
            Vector21.Zero,
            Matrix21.Zero,
            Matrix21.Zero,
            Vector21.Zero,
            Vector21.Zero,
            Vector21.Zero,
            Vector21.Zero,
            Vector21.Zero);


    }
    public struct FrameArticulated
    {
        public FrameArticulated(
            in Matrix21 articulatedInertia,
            in Vector21 articulatedForces,
            in Vector21 precussionAxis,
            in Matrix21 reactionSpace) : this()
        {
            ArticulatedInertia = articulatedInertia;
            ArticulatedForces = articulatedForces;
            PrecussionAxis = precussionAxis;
            ReactionSpace = reactionSpace;
        }
        public Matrix21 ArticulatedInertia { get; }
        public Vector21 ArticulatedForces { get; }
        public Vector21 PrecussionAxis { get; }
        public Matrix21 ReactionSpace { get; }
    }

}
