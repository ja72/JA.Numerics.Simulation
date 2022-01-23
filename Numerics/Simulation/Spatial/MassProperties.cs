using System;
using System.ComponentModel;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class MassProperties : IHasUnits<MassProperties>
    {
        public static readonly MassProperties Zero = new MassProperties(UnitSystem.SI, 0, Matrix3.Zero, Vector3.Zero);
        internal readonly (float mass, Matrix3 mmoi, Vector3 cg) data;

        #region Factory
        public MassProperties(UnitSystem units, float mass, Matrix3 mmoi, Vector3 cg) : this(units, (mass, mmoi, cg)) { }
        public MassProperties(UnitSystem units, (float mass, Matrix3 mmoi, Vector3 cg) data) 
        {
            this.Units = units;
            this.data = data;
        }
        public static MassProperties Cylinder(UnitSystem units, Vector3 cg, float mass, float radius, float height)
            => new MassProperties(units,
                mass,
                Matrix3.Diagonal(
                    mass / 4 * radius * radius + mass / 12 * height * height,
                    mass / 4 * radius * radius + mass / 12 * height * height,
                    mass / 2 * radius * radius),
                cg);

        public static MassProperties Sphere(UnitSystem units, Vector3 cg, float mass, float radius)
            => new MassProperties(units,
                mass,
                Matrix3.Diagonal(
                    2 * mass / 5 * radius * radius,
                    2 * mass / 5 * radius * radius,
                    2 * mass / 5 * radius * radius),
                cg);

        public static MassProperties Box(UnitSystem units, Vector3 cg, float mass, float width, float height, float thickness)
            => new MassProperties(units,
                mass,
                Matrix3.Diagonal(
                    mass / 12 * (thickness * thickness + height * height),
                    mass / 12 * (width * width + thickness * thickness),
                    mass / 12 * (width * width + height * height)),
                cg); 
        #endregion

        #region Properties
        public UnitSystem Units { get; }
        public float Mass
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.mass;
        }
        public Matrix3 MMoi
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.mmoi;
        }
        public Vector3 CG
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => data.cg;
        }

        public Vector3 GetPrincipalMmoi(out Pose pose)
        {
            var w = MMoi.GetEigenValues();
            var X = MMoi.GetEigenVectors(w);
            var q = X.ToRotation();
            pose = new Pose(CG, q);
            return w;
        }
        #endregion

        #region Dynamics
        public Vector33 GetWeight(Vector3 gravity, Vector3 cg)
        {
            return Vector33.Wrench(data.mass * gravity, cg, 0);
        }
        public Vector33 GetWeight(Pose pose, Vector3 gravity)
        {
            var cg = Pose.FromLocal(pose, data.cg);
            return GetWeight(gravity, cg);
        }

        public Matrix33 Spi(Quaternion orientation, Vector3 cg)
        {
            //tex: Spatial inertia matrix
            //$$\mathbf{I} = \begin{bmatrix}m & -m\,c\times\\
            //m\,c\times & I_{C}-m\,c\times c\times
            //\end{bmatrix}$$
            var cgx = cg.Cross();
            var R = Matrix3.CreateRotation(orientation);
            var I_c = R * data.mmoi * R.Transpose();

            return new Matrix33(
                data.mass, -data.mass * cgx,
                data.mass * cgx, I_c + data.mass * cg.Mmoi());
        }
        public Matrix33 Spi(Pose pose)
        {
            return Spi(pose.Orientation, Pose.FromLocal(pose, data.cg));
        }

        public Matrix33 Spm(Quaternion orientation, Vector3 cg)
        {
            //tex: Inverse spatial inertia matrix
            // $$\mathbf{I}^{-1}=\begin{bmatrix}\frac{1}{m}-c\times I_{C}^{-1}c\times & c\times I_{C}^{-1}\\
            //-I_{C}^{-1}c\times & I_{C}^{-1}
            //\end{bmatrix}$$
            var cgx = cg.Cross();
            var R = Matrix3.CreateRotation(orientation);
            var M_c = R * data.mmoi.Inverse() * R.Transpose();

            return new Matrix33(
                1 / data.mass - cgx * M_c * cgx, cgx * M_c,
                -M_c * cgx, M_c);
        }
        public Matrix33 Spm(Pose pose)
        {
            return Spm(pose.Orientation, Pose.FromLocal(pose, data.cg));
        }
        #endregion

        #region Algebra
        public static MassProperties Scale(float factor, MassProperties a)
        {
            return new MassProperties(a.Units, factor * a.Mass, factor * a.MMoi, a.CG);
        }

        public static MassProperties Add(MassProperties a, MassProperties b)
        {
            if (a.Units != b.Units) throw new NotSupportedException();
            float m = a.Mass + b.Mass;
            var cg = (a.CG * a.Mass + b.CG * b.Mass)/m;
            var mmoi = a.MMoi + a.Mass * a.CG.Mmoi() + b.MMoi + b.Mass * b.CG.Mmoi();
            return new MassProperties(a.Units, m, mmoi - m * cg.Mmoi(), cg);
        }
        public static MassProperties Subtract(MassProperties a, MassProperties b)
        {
            if (a.Units != b.Units) throw new NotSupportedException();
            float m = a.Mass - b.Mass;
            var cg = (a.CG * a.Mass - b.CG * b.Mass) / m;
            var mmoi = a.MMoi + a.Mass * a.CG.Mmoi() - b.MMoi - b.Mass * b.CG.Mmoi();
            return new MassProperties(a.Units, m, mmoi - m * cg.LengthSquared(), cg);
        }

        public static MassProperties operator +(MassProperties a, MassProperties b) => Add(a, b);
        public static MassProperties operator -(MassProperties a, MassProperties b) => Subtract(a, b);
        public static MassProperties operator *(float factor, MassProperties a) => Scale(factor, a);
        public static MassProperties operator *(MassProperties a, float factor) => Scale(factor, a);
        public static MassProperties operator /(MassProperties a, float divisor) => Scale(1/divisor, a); 
        #endregion

        #region Formatting
        public override string ToString()
            => $"Body(Units={Units}, Mass={Mass:g3}, MMOI={MMoi:g3}, CG={CG:g3})";

        #endregion

        #region Units
        public MassProperties ConvertTo(UnitSystem target)
        {
            float fm = UnitFactors.Mass(Units, target);
            float fl = UnitFactors.Length(Units, target);
            float fi = fm * UnitFactors.Area(Units, target);
            return new MassProperties(target,
                (fm * data.mass, fi * data.mmoi, fl * data.cg));
        } 
        #endregion
    }
}
