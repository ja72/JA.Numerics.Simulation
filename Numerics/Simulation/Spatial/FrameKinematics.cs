using System.ComponentModel;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public struct FrameKinematics
    {
        public FrameKinematics(
            in JointInfo joint,
            in Pose position,
            in Vector3 cgPosition,
            in Vector33 jointAxis,
            in Vector33 velocity,
            in Vector33 biasAcceleration,
            in Matrix33 spatialInertia,
            in Matrix33 spatialMobility,
            in Vector33 momentum,
            in Vector33 biasForce,
            in Vector33 appliedForce,
            in Vector33 acceleration,
            in Vector33 force) : this()
        {
            Joint = joint;
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
        public JointInfo Joint { get; }
        public Pose Position { get; }
        public Vector3 CgPosition { get; }
        public Vector33 JointAxis { get; }
        public Vector33 Velocity { get; }
        public Vector33 BiasAcceleration { get; }
        public Matrix33 SpatialInertia { get; }
        public Matrix33 SpatialMobility { get; }
        public Vector33 Momentum { get; }
        public Vector33 BiasForce { get; }
        public Vector33 AppliedForce { get; }
        public Vector33 Acceleration { get; set; }
        public Vector33 Force { get; set; }
        public Vector3 MaterialAccelerationAt(Vector3 target)
        {
            return MaterialAcceleration(
                Acceleration.TwistAt(target),
                Velocity.TwistAt(target), 
                Velocity.Vector2);
        }
        public Vector3 CgAccleration { get => MaterialAccelerationAt(CgPosition); }

        public Vector33 GetInertialForce()
        {
            return BiasForce + SpatialInertia * Acceleration;
        }

        public static readonly FrameKinematics Zero = new FrameKinematics(
            JointInfo.Default,
            Pose.Origin,
            Vector3.Zero,
            Vector33.Zero,
            Vector33.Zero,
            Vector33.Zero,
            Matrix33.Zero,
            Matrix33.Zero,
            Vector33.Zero,
            Vector33.Zero,
            Vector33.Zero,
            Vector33.Zero,
            Vector33.Zero);
        public static Vector3 MaterialAcceleration(Vector3 spatialAcceleration, Vector3 velocity, Vector3 omega)
            => spatialAcceleration + LinearAlgebra.Cross(omega, velocity);
        public static Vector3 SpatialAcceleration(Vector3 materialAcceleration, Vector3 velocity, Vector3 omega)
            => materialAcceleration - LinearAlgebra.Cross(omega, velocity);
    }

    public struct FrameArticulated
    {
        public FrameArticulated(
            in Matrix33 articulatedInertia,
            in Vector33 articulatedForces,
            in Vector33 precussionAxis,
            in Matrix33 reactionSpace) : this()
        {
            ArticulatedInertia = articulatedInertia;
            ArticulatedForces = articulatedForces;
            PrecussionAxis = precussionAxis;
            ReactionSpace = reactionSpace;
        }
        public Matrix33 ArticulatedInertia { get; }
        public Vector33 ArticulatedForces { get; }
        public Vector33 PrecussionAxis { get; }
        public Matrix33 ReactionSpace { get; }
    }

}
