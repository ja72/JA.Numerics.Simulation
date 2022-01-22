using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Xml;

namespace JA.Numerics.Simulation.Spatial
{
    public struct Pose : ICanConvert<Pose>
    {
        public Pose(Vector3 position, Quaternion orientation) : this()
        {
            Position = position;
            Orientation = orientation;
        }
        public static Pose At(Vector3 position) => new Pose(position, Quaternion.Identity);
        public static Pose About(Quaternion orientation) => new Pose(Vector3.Zero, orientation);

        public static implicit operator Pose(Vector3 position) => At(position);
        public static implicit operator Pose(Quaternion orientation) => About(orientation);
        public static readonly Pose Origin = new Pose(Vector3.Zero, Quaternion.Identity);
        public static Pose AlongX(float distance) => new Pose(Vector3.UnitX * distance, Quaternion.Identity);
        public static Pose AlongY(float distance) => new Pose(Vector3.UnitY * distance, Quaternion.Identity);
        public static Pose AlongZ(float distance) => new Pose(Vector3.UnitZ * distance, Quaternion.Identity);
        public static Pose AboutX(float angle) => new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitX, angle));
        public static Pose AboutY(float angle) => new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitY, angle));
        public static Pose AboutZ(float angle) => new Pose(Vector3.Zero, Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angle));

        public Vector3 Position { get; }
        public Quaternion Orientation { get; }

        #region Transforms
        public static Pose FromLocal(Pose parent, Pose local)
            => new Pose(
                parent.Position + local.Position.Rotate(parent.Orientation),
                parent.Orientation * local.Orientation);
        public static Pose ToLocal(Pose world, Pose parent)
            => new Pose(
                world.Position.Rotate(-parent.Orientation) - parent.Position,
                world.Orientation / parent.Orientation);
        public static Vector3 FromLocal(Pose parent, Vector3 local)
            => parent.Position + local.Rotate(parent.Orientation);
        public static Vector3 ToLocal(Vector3 world, Pose parent)
            => world.Rotate(-parent.Orientation) - parent.Position;
        public static Vector3 FromLocalDirection(Pose parent, Vector3 local)
            => local.Rotate(parent.Orientation);
        public static Vector3 ToLocalDirection(Vector3 world, Pose parent)
            => world.Rotate(-parent.Orientation);

        public static Quaternion FromLocal(Pose parent, Quaternion local)
            => parent.Orientation * local;
        public static Quaternion ToLocal(Quaternion world, Pose parent)
            => world / parent.Orientation;
        #endregion

        #region Algebra
        public static Pose Negate(Pose a)
            => new Pose(
                -a.Position,
                -a.Orientation);
        public static Pose Scale(float factor, Pose a)
            => new Pose(
                factor * a.Position,
                Quaternion.Multiply(a.Orientation, factor));
        public static Pose Add(Pose a, Pose b)
            => new Pose(
                a.Position + b.Position,
                a.Orientation + b.Orientation);
        public static Pose Subtract(Pose a, Pose b)
            => new Pose(
                a.Position - b.Position,
                a.Orientation - b.Orientation);

        public static Pose operator +(Pose a, Pose b) => Add(a, b);
        public static Pose operator -(Pose a) => Negate(a);
        public static Pose operator -(Pose a, Pose b) => Subtract(a, b);
        public static Pose operator *(float f, Pose a) => Scale(f, a);
        public static Pose operator *(Pose a, float f) => Scale(f, a);
        public static Pose operator /(Pose a, float d) => Scale(1 / d, a);

        public static Pose operator *(Pose parent, Pose local) => FromLocal(parent, local);
        public static Pose operator /(Pose world, Pose parent) => ToLocal(world, parent);
        public static Vector3 operator *(Pose parent, Vector3 local) => FromLocal(parent, local);
        public static Vector3 operator /(Vector3 world, Pose parent) => ToLocal(world, parent);
        public static Quaternion operator *(Pose parent, Quaternion local) => FromLocal(parent, local);
        public static Quaternion operator /(Quaternion world, Pose parent) => ToLocal(world, parent);
        #endregion


        public override string ToString() => $"Pose({{{Position}|{Orientation}}})";

        public Pose ConvertFromTo(UnitSystem units, UnitSystem target)
        {
            if (units == target) return this;
            float fl = UnitFactors.Length(units, target);
            return new Pose(fl * Position, Orientation);
        }

        public Pose CalcRate(Vector33 motion)
        {
            return new Pose(motion.Vector1, Orientation.Derivative(motion.Vector2));
        }
    }
}


