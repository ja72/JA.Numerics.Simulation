using System.Numerics;
using System.Runtime.CompilerServices;

namespace JA.Numerics.Simulation.Planar
{
    public struct MassProperties
    {
        public static readonly MassProperties Zero = new MassProperties(0, 0, Vector2.Zero);
        internal readonly (float mass, float mmoi, Vector2 cg) data;

        public MassProperties(float mass, float mmoi, Vector2 cg) : this((mass, mmoi, cg)) { }
        public MassProperties((float mass, float mmoi, Vector2 cg) data) : this()
        {
            this.data = data;
        }

        public float Mass
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.mass;
        }
        public float MMoi
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.mmoi;
        }
        public Vector2 CG
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.cg;
        }
        public static MassProperties Circle(Vector2 cg, float mass, float radius)
            => new MassProperties(mass, mass / 2 * radius * radius, cg);

        public static MassProperties Sphere(Vector2 cg, float mass, float radius)
            => new MassProperties(mass, 2 * mass / 5 * radius * radius, cg);

        public static MassProperties Rectangle(Vector2 cg, float mass, float width, float height)
            => new MassProperties(mass, mass / 12 * (width * width + height * height), cg);

        public static MassProperties Scale(float factor, MassProperties a)
        {
            return new MassProperties(factor * a.Mass, factor * a.MMoi, a.CG);
        }

        public static MassProperties Add(MassProperties a, MassProperties b)
        {
            float m = a.Mass + b.Mass;
            Vector2 cg = (a.CG * a.Mass + b.CG * b.Mass)/m;
            float mmoi = a.MMoi + a.Mass * a.CG.LengthSquared() + b.MMoi + b.Mass * b.CG.LengthSquared();
            return new MassProperties(m, mmoi - m * cg.LengthSquared(), cg);
        }
        public static MassProperties Subtract(MassProperties a, MassProperties b)
        {
            float m = a.Mass - b.Mass;
            Vector2 cg = (a.CG * a.Mass - b.CG * b.Mass) / m;
            float mmoi = a.MMoi + a.Mass * a.CG.LengthSquared() - b.MMoi - b.Mass * b.CG.LengthSquared();
            return new MassProperties(m, mmoi - m * cg.LengthSquared(), cg);
        }

        public static MassProperties operator +(MassProperties a, MassProperties b) => Add(a, b);
        public static MassProperties operator -(MassProperties a, MassProperties b) => Subtract(a, b);
        public static MassProperties operator *(float factor, MassProperties a) => Scale(factor, a);
        public static MassProperties operator *(MassProperties a, float factor) => Scale(factor, a);
        public static MassProperties operator /(MassProperties a, float divisor) => Scale(1/divisor, a);

        #region Formatting
        public override string ToString() 
            => $"Body(Mass={Mass}, MMOI={MMoi}, CG={CG})";
        #endregion
    }
}
