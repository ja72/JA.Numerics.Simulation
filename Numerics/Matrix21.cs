using System;
using System.Numerics;


namespace JA.Numerics
{
    public struct Matrix21
    {
        internal readonly (Matrix2 m_11, Vector2 m_12, Vector2 m_21, float m_22) data;

        public Matrix21(Matrix2 m_11, Vector2 m_12, Vector2 m_21, float m_22)
            : this((m_11, m_12, m_21, m_22)) { }
        public Matrix21((Matrix2 m_11, Vector2 m_12, Vector2 m_21, float m_22) data)
            : this()
        {
            this.data = data;
        }
        public static readonly Matrix21 Zero = new Matrix21(Matrix2.Zero, Vector2.Zero, Vector2.Zero, 0);
        public static implicit operator Matrix21(float d) => Diagonal(d);
        public static Matrix21 Diagonal(float d) => Diagonal(new Vector2(d, d), d);
        public static Matrix21 Diagonal(Vector2 v, float s) => new Matrix21(Matrix2.Diagonal(v), Vector2.Zero, Vector2.Zero, s);
        public static Matrix21 Symmetric(Matrix2 matrix, Vector2 vector, float scalar)
            => new Matrix21(matrix, vector, vector, scalar);
        public Matrix2 Matrix { get => data.m_11; }
        public Vector2 Vector1 { get => data.m_12; }
        public Vector2 Vector2 { get => data.m_21; }
        public float Scalar { get => data.m_22; }

        public static Matrix21 Negate(Matrix21 A)
            => new Matrix21(
                -A.data.m_11,
                -A.data.m_12,
                -A.data.m_21,
                -A.data.m_22);
        public static Matrix21 Scale(float f, Matrix21 A)
            => new Matrix21(
                f * A.data.m_11,
                f * A.data.m_12,
                f * A.data.m_21,
                f * A.data.m_22);
        public static Matrix21 Add(Matrix21 A, Matrix21 B)
            => new Matrix21(
                A.data.m_11 + B.data.m_11,
                A.data.m_12 + B.data.m_12,
                A.data.m_21 + B.data.m_21,
                A.data.m_22 + B.data.m_22);
        public static Matrix21 Subtract(Matrix21 A, Matrix21 B)
            => new Matrix21(
                A.data.m_11 - B.data.m_11,
                A.data.m_12 - B.data.m_12,
                A.data.m_21 - B.data.m_21,
                A.data.m_22 - B.data.m_22);

        public Matrix21 Transpose()
            => new Matrix21(data.m_11.Transpose(), data.m_21, data.m_12, data.m_22);

        public static Matrix21 operator -(Matrix21 A) => Negate(A);
        public static Matrix21 operator +(Matrix21 A, Matrix21 B) => Add(A, B);
        public static Matrix21 operator -(Matrix21 A, Matrix21 B) => Subtract(A, B);
        public static Matrix21 operator *(float f, Matrix21 A) => Scale(f, A);
        public static Matrix21 operator *(Matrix21 A, float f) => Scale(f, A);
        public static Matrix21 operator /(Matrix21 A, float d) => Scale(1 / d, A);
        public static Matrix21 operator ~(Matrix21 A) => A.Transpose();

        public static Vector21 Product(Matrix21 I, Vector21 t)
            => new Vector21(
                I.data.m_11 * t.data.m_1 + I.data.m_12 * t.data.m_2,
                LinearAlgebra.Dot(I.data.m_21, t.data.m_1) + I.data.m_22 * t.data.m_2);

        public static Matrix21 Product(Matrix21 M, Matrix21 I) 
            => new Matrix21(
                M.data.m_11 * I.data.m_11 + LinearAlgebra.Outer(M.data.m_12, I.data.m_21),
                M.data.m_11 * I.data.m_12 + M.data.m_12 * I.data.m_22,
                M.data.m_21 * I.data.m_11 + M.data.m_22 * I.data.m_21,
                LinearAlgebra.Dot(M.data.m_21, I.data.m_12) + (M.data.m_22 * I.data.m_22));

        public static Vector21 operator *(Matrix21 I, Vector21 v) => Product(I, v);
        public static Matrix21 operator *(Matrix21 M, Matrix21 I) => Product(M, I);

        public Vector21 Solve(Vector21 v) => Inverse() * v;
        public Matrix21 Solve(Matrix21 M) => Inverse() * M;

        public Matrix21 Inverse()
        {
            Matrix2 M = data.m_11, Mt = data.m_11.Transpose();
            Vector2 v1 = data.m_12, v2 = data.m_21;
            float s = data.m_22;

            Vector2 u1 = (M - LinearAlgebra.Outer(v1, v2) / s).Solve(v1 / (-s));
            Vector2 r2 = Mt.Solve(v2);
            Vector2 u2 = r2 / (LinearAlgebra.Dot(v1, r2) - s);
            Matrix2 R = M.Solve(1 - LinearAlgebra.Outer(v1, u2));
            float t = (1 - LinearAlgebra.Dot(v2, u1)) / s;

            return new Matrix21(R, u1, u2, t);
        }
        public static Vector21 operator /(Vector21 v, Matrix21 A) => A.Solve(v);
        public static Matrix21 operator /(Matrix21 B, Matrix21 A) => A.Solve(B);
        public static Matrix21 operator !(Matrix21 A) => A.Inverse();

        public override string ToString() => $"Matrix21(" +
            $"{data.m_11.data.m_11},{data.m_11.data.m_12},{data.m_12.X}|" +
            $"{data.m_11.data.m_21},{data.m_11.data.m_22},{data.m_12.Y}|" +
            $"{data.m_21.X},{data.m_21.Y},{data.m_22})";

    }
}

