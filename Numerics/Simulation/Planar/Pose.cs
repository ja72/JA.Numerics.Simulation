using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics.Simulation.Planar
{
    public struct Pose
    {
        public Pose(float orientation) : this(Vector2.Zero, orientation) { }
        public Pose(Vector2 position) : this(position, 0) { }
        public Pose(Vector2 position, float orientation) : this()
        {
            Position = position;
            Orientation = orientation;
        }
        public static implicit operator Pose(Vector2 position) => new Pose(position);
        public static implicit operator Pose(float orientation) => new Pose(orientation);
        public static readonly Pose Origin = new Pose(Vector2.Zero, 0);
        public static Pose AlongX(float distance) => new Pose(Vector2.UnitX * distance, 0);
        public static Pose AlongY(float distance) => new Pose(Vector2.UnitY * distance, 0);
        public static Pose AboutZ(float angle) => new Pose(Vector2.Zero, angle);
        public Vector2 Position { get; }
        public float Orientation { get; }

        #region Algebra
        public static Pose FromLocal(Pose parent, Pose local) 
            => new Pose(
                parent.Position + local.Position.Rotate(parent.Orientation),
                parent.Orientation + local.Orientation);
        public static Pose ToLocal(Pose world, Pose parent) 
            => new Pose(
                world.Position.Rotate(-parent.Orientation) - parent.Position,
                world.Orientation - parent.Orientation);
        public static Pose operator +(Pose parent, Pose local) => FromLocal(parent, local);
        public static Pose operator -(Pose world, Pose parent) => ToLocal(world, parent);
        #endregion

        public override string ToString() => $"Pose({{{Position:g3}|{Orientation:g3}}})";

    }

}


