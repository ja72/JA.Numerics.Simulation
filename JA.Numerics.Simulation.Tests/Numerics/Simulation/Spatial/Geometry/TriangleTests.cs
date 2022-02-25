using Microsoft.VisualStudio.TestTools.UnitTesting;
using JA.Numerics.Simulation.Spatial.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial.Geometry
{
    [TestClass()]
    public class TriangleTests
    {
        [TestMethod()]
        public void TriangleTest()
        {
            var A = new Vector3(10f.Random(), -5f.Random(), -5f.Random());
            var B = new Vector3(-5f.Random(), 10f.Random(), -5f.Random());
            var C = new Vector3(-5f.Random(), -5f.Random(), 10f.Random());

            var triangle = new Triangle(A, B, C);

            Assert.AreEqual(A, triangle.A);
            Assert.AreEqual(B, triangle.B);
            Assert.AreEqual(C, triangle.C);

            Assert.AreEqual(Vector3.Distance(A, B), triangle.SideAB.Length);
            Assert.AreEqual(Vector3.Distance(B, C), triangle.SideBC.Length);
            Assert.AreEqual(Vector3.Distance(C, A), triangle.SideCA.Length);
        }

        [TestMethod()]
        public void DistanceToTest()
        {
            const float tol = LinearAlgebra.ZeroTolerance;

            var triangle = new Triangle( Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ );

            Assert.AreEqual(1f, triangle.DistanceTo(2*Vector3.UnitX), (float)tol);
            Assert.AreEqual(1f, triangle.DistanceTo(2*Vector3.UnitY), (float)tol);
            Assert.AreEqual(1f, triangle.DistanceTo(2*Vector3.UnitZ), (float)tol);

            Assert.AreEqual(3f.Sqrt()/2, triangle.DistanceTo(Vector3.Zero), (float)tol);
            Assert.AreEqual(3f.Sqrt()/2, triangle.DistanceTo(Vector3.One), (float)tol);
        }

        [TestMethod()]
        public void ProjectTest()
        {
            var triangle = new Triangle(Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ);

            var cen = triangle.ProjectOnPlane(Vector3.Zero);
            CollectionAssert.AreEqual(triangle.Center.ToArray(), cen.ToArray(), LinearAlgebra.AbsFloat(1e-5f));
        }

        [TestMethod()]
        public void BarycentricTest()
        {
            var A = new Vector3(10f.Random(), -5f.Random(), -5f.Random());
            var B = new Vector3(-5f.Random(), 10f.Random(), -5f.Random());
            var C = new Vector3(-5f.Random(), -5f.Random(), 10f.Random());

            var triangle = new Triangle(A, B, C);

            for (int i = 0; i < 10; i++)
            {
                var point = new Vector3(5f.Random()-5f.Random(), 5f.Random()-5f.Random(), 5f.Random()-5f.Random());

                point = triangle.ProjectOnPlane(point);

                if (triangle.Barycentric(point, out var coord))
                {
                    var test = triangle.GetPoint(coord);

                    CollectionAssert.AreEqual(point.ToArray(), test.ToArray(), LinearAlgebra.AbsFloat(1e-4f));
                }
            }
        }

        [TestMethod()]
        public void ContainsTest()
        {
            var A = new Vector3(10f.Random(), -5f.Random(), -5f.Random());
            var B = new Vector3(-5f.Random(), 10f.Random(), -5f.Random());
            var C = new Vector3(-5f.Random(), -5f.Random(), 10f.Random());

            var triangle = new Triangle(A, B, C);

            Assert.IsTrue(triangle.Contains(triangle.GetPoint(0.2f, 0.3f, 0.5f)));
            Assert.IsTrue(triangle.Contains(triangle.GetPoint(0.3f, 0.5f, 0.2f)));
            Assert.IsTrue(triangle.Contains(triangle.GetPoint(0.5f, 0.2f, 0.3f)));

            Assert.IsFalse(triangle.Contains(triangle.GetPoint(-0.2f, 0.8f, 0.4f)));
            Assert.IsFalse(triangle.Contains(triangle.GetPoint(0.8f, 0.4f, -0.2f)));
            Assert.IsFalse(triangle.Contains(triangle.GetPoint(0.4f, -0.2f, 0.8f)));

            Assert.IsFalse(triangle.Contains(triangle.GetPoint(0.2f, 0.3f, 0.5f)+triangle.Normal));
            Assert.IsFalse(triangle.Contains(triangle.GetPoint(0.3f, 0.5f, 0.2f)+triangle.Normal));
            Assert.IsFalse(triangle.Contains(triangle.GetPoint(0.5f, 0.2f, 0.3f)+triangle.Normal));
        }

        [TestMethod()]
        public void ScaleTest()
        {
            var triangle = new Triangle(
                Vector3.UnitX,
                Vector3.UnitY,
                Vector3.UnitZ);

            var twice = 2f*triangle;
            var minus = -triangle;

            Assert.AreEqual(2*Vector3.UnitX, twice.A);
            Assert.AreEqual(2*Vector3.UnitY, twice.B);
            Assert.AreEqual(2*Vector3.UnitZ, twice.C);

            Assert.AreEqual(-Vector3.UnitX, minus.A);
            Assert.AreEqual(-Vector3.UnitY, minus.B);
            Assert.AreEqual(-Vector3.UnitZ, minus.C);

        }

        [TestMethod()]
        public void OffsetTest()
        {
            var triangle = new Triangle(
                Vector3.UnitX,
                Vector3.UnitY,
                Vector3.UnitZ);

            var plus = triangle + Vector3.UnitX;
            Assert.AreEqual(2*Vector3.UnitX, plus.A);
            Assert.AreEqual(Vector3.UnitY+Vector3.UnitX, plus.B);
            Assert.AreEqual(Vector3.UnitZ+Vector3.UnitX, plus.C);

            var minus = triangle - Vector3.UnitX;
            Assert.AreEqual(Vector3.Zero, minus.A);
            Assert.AreEqual(Vector3.UnitY-Vector3.UnitX, minus.B);
            Assert.AreEqual(Vector3.UnitZ-Vector3.UnitX, minus.C);
        }

        [TestMethod()]
        public void TransformTest()
        {
            var triangle = new Triangle(
                Vector3.UnitX,
                Vector3.UnitY,
                Vector3.UnitZ);

            var transform = Matrix3.FromColumns(
                Vector3.UnitY,
                Vector3.UnitZ,
                Vector3.UnitX);

            var target = triangle.Transform(transform);

            Assert.AreEqual(Vector3.UnitY, target.A);
            Assert.AreEqual(Vector3.UnitZ, target.B);
            Assert.AreEqual(Vector3.UnitX, target.C);
        }

        [TestMethod()]
        public void RotateTest()
        {
            var triangle = new Triangle(
                Vector3.UnitX,
                Vector3.UnitY,
                Vector3.UnitZ);

            var rotate = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.5f.Pi());

            var target = triangle.Rotate(rotate);

            Assert.AreEqual(Vector3.UnitX, target.A);
            Assert.AreEqual(Vector3.UnitZ, target.B);
            Assert.AreEqual(-Vector3.UnitY, target.C);
        }

        [TestMethod()]
        public void ConvertFromToTest()
        {
            var triangle = new Triangle(
                25.4f*Vector3.UnitX,
                25.4f*Vector3.UnitY,
                25.4f*Vector3.UnitZ);

            var target = triangle.ConvertFromTo(UnitSystem.MMKS, UnitSystem.IPS);

            var cmp = LinearAlgebra.AbsFloat(1e-5f);

            CollectionAssert.AreEqual(Vector3.UnitX.ToArray(), target.A.ToArray(), cmp);
            CollectionAssert.AreEqual(Vector3.UnitY.ToArray(), target.B.ToArray(), cmp);
            CollectionAssert.AreEqual(Vector3.UnitZ.ToArray(), target.C.ToArray(), cmp);
        }
    }
}