using System;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial.Geometry
{
    using JA.Numerics.Simulation;

    public readonly struct Circle
    {
        public Circle(Vector3 center, Vector3 normal, float radius) : this()
        {
            Center=center;
            Normal=normal.Unit();
            Radius=radius;
        }

        public Vector3 Center { get; }
        public Vector3 Normal { get; }
        public float Radius { get; }
        public float Area { get => Radius.Sqr().Pi(); }
        public bool Contains(Vector3 point)
        {
            float z = Vector3.Dot(Normal, point-Center);
            if (Math.Abs(z)<=Constants.DistanceTolerance)
            {
                // Point is on plane
                float d = Vector3.Distance(point, Center);

                return d<=Radius;
            }
            return false;
        }

        public static Circle Circumscribed(Vector3 A, Vector3 B, Vector3 C)
            => Circumscribed(new Triangle(A, B, C));    
        public static Circle Circumscribed(Triangle triangle)
        {
            var t = triangle.B - triangle.A;
            var u = triangle.C - triangle.A;
            var v = triangle.C - triangle.B;

            var n = Vector3.Cross(t, u);
            var n_sq = Vector3.Dot(n, n);

            if (n_sq < LinearAlgebra.ZeroTolerance)
            {

                throw new ArgumentException("Three points are colinear.", nameof(triangle));
            }

            var insq2 = 1/(2*n_sq);
            var tt = Vector3.Dot(t, t);
            var uu = Vector3.Dot(u, u);

            var cen = triangle.A + (u*tt*Vector3.Dot(u, v) - t*uu*Vector3.Dot(t, v))*insq2;
            var r = Constants.Sqrt(tt*uu*Vector3.Dot(v, v)*insq2*0.5f);

            return new Circle(cen, n, r);
        }

        #region Algebra
        public Circle Scale(float factor)
        {
            return new Circle(factor*Center, Normal, factor*Radius);
        }
        public Circle Offset(Vector3 offset)
        {
            return new Circle(Center+offset, Normal, Radius);
        }
        public Circle Transform(Matrix3 transform)
        {
            return new Circle(transform*Center, transform*Normal, Radius);
        }
        public Circle Rotate(Quaternion rotation)
        {
            return new Circle(Center.Rotate(rotation), Normal.Rotate(rotation), Radius);
        }
        public static Circle operator +(Circle Circle, Vector3 offset) => Circle.Offset(offset);
        public static Circle operator +(Vector3 offset, Circle Circle) => Circle.Offset(offset);
        public static Circle operator -(Circle Circle, Vector3 offset) => Circle.Offset(-offset);
        public static Circle operator -(Vector3 offset, Circle Circle) => Circle.Scale(-1f).Offset(offset);

        public static Circle operator *(float factor, Circle Circle) => Circle.Scale(factor);
        public static Circle operator *(Circle Circle, float factor) => Circle.Scale(factor);
        public static Circle operator /(Circle Circle, float divisor) => Circle.Scale(1/divisor);

        #endregion

        public Circle ConvertFromTo(UnitSystem units, UnitSystem target)
        {
            var fl = Unit.Length.Convert(units, target);

            return new Circle(fl*Center, Normal, fl*Radius);
        }

    }
}
