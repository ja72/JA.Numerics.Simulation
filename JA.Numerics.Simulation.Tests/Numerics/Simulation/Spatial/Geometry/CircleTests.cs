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
    public class CircleTests
    {
        [TestMethod()]
        public void CircleTest()
        {
            var circle = new Circle(2*Vector3.UnitX+Vector3.UnitY, 2*Vector3.UnitZ, 2);

            Assert.AreEqual(2*Vector3.UnitX+Vector3.UnitY, circle.Center);
            Assert.AreEqual(Vector3.UnitZ, circle.Normal);
            Assert.AreEqual(2f, circle.Radius);
        }

        [TestMethod()]
        public void ContainsTest()
        {
            var circle = new Circle(Vector3.UnitX, Vector3.UnitY, 1);

            Assert.IsTrue(circle.Contains(Vector3.Zero));
            Assert.IsTrue(circle.Contains(2*Vector3.UnitX));
            Assert.IsTrue(circle.Contains(Vector3.UnitX + Vector3.UnitZ));
            Assert.IsTrue(circle.Contains(Vector3.UnitX - Vector3.UnitZ));

            var circle2 = new Circle(2*Vector3.UnitX+Vector3.UnitZ, 2*Vector3.UnitZ, 2);

            Assert.IsFalse(circle2.Contains(Vector3.Zero));
            Assert.IsTrue(circle2.Contains(Vector3.UnitZ));
            Assert.IsTrue(circle2.Contains(circle2.Center));
            Assert.IsFalse(circle2.Contains(2*circle2.Center));
            Assert.IsTrue(circle2.Contains(circle2.Center+Vector3.UnitX));
            Assert.IsTrue(circle2.Contains(circle2.Center+2*Vector3.UnitX));
            Assert.IsFalse(circle2.Contains(circle2.Center+3*Vector3.UnitX));

        }

        [TestMethod()]
        public void CircumscribedTest()
        {            

            var A = new Vector3(10f.Random(), -5f.Random(), -5f.Random());
            var B = new Vector3(-5f.Random(), 10f.Random(), -5f.Random());
            var C = new Vector3(-5f.Random(), -5f.Random(), 10f.Random());

            var circle = Circle.Circumscribed(A, B, C);

            Assert.IsTrue(circle.Contains((A+B+C)/3));
            Assert.IsTrue(circle.Contains(A));
            Assert.IsTrue(circle.Contains(B));
            Assert.IsTrue(circle.Contains(C));

            var N = Vector3.Cross(A, B) + Vector3.Cross(B, C) + Vector3.Cross(C, A);
            CollectionAssert.AreEqual(Vector3.Normalize(N).ToArray(), circle.Normal.ToArray(), LinearAlgebra.AbsFloat(1e-5f));

        }


        [TestMethod()]
        public void ScaleTest()
        {
            var circle = new Circle(Vector3.UnitX, Vector3.UnitY, 1);

            circle = 2f*circle;

            Assert.IsTrue(circle.Contains(Vector3.Zero));
            Assert.IsTrue(circle.Contains(4*Vector3.UnitX));
            Assert.IsTrue(circle.Contains(2*Vector3.UnitX + 2*Vector3.UnitZ));
            Assert.IsTrue(circle.Contains(2*Vector3.UnitX - 2*Vector3.UnitZ));

        }

        [TestMethod()]
        public void OffsetTest()
        {
            var circle = new Circle(Vector3.UnitX, Vector3.UnitY, 1);

            circle += Vector3.UnitY;

            Assert.IsTrue(circle.Contains(Vector3.UnitY));
            Assert.IsTrue(circle.Contains(2*Vector3.UnitX+Vector3.UnitY));
            Assert.IsTrue(circle.Contains(Vector3.UnitX + Vector3.UnitZ+Vector3.UnitY));
            Assert.IsTrue(circle.Contains(Vector3.UnitX - Vector3.UnitZ+Vector3.UnitY));

        }

        [TestMethod()]
        public void TransformTest()
        {
            var circle = new Circle(Vector3.UnitX, Vector3.UnitY, 1);

            // create a transformation to permutate axis
            var transform = Matrix3.FromColumns(
                Vector3.UnitY,
                Vector3.UnitZ,
                Vector3.UnitX);

            circle = circle.Transform(transform);

            Assert.AreEqual(Vector3.UnitY, circle.Center);
            Assert.AreEqual(Vector3.UnitZ, circle.Normal);
            Assert.AreEqual(1f, circle.Radius);
        }

        [TestMethod()]
        public void RotateTest()
        {
            var circle = new Circle(Vector3.UnitX, Vector3.UnitY, 1);

            var rotate = Quaternion.CreateFromAxisAngle(Vector3.UnitX, 0.5f.Pi());

            circle = circle.Rotate(rotate);

            CollectionAssert.AreEqual(Vector3.UnitX.ToArray(), circle.Center.ToArray(), LinearAlgebra.AbsFloat(1e-7f));
            CollectionAssert.AreEqual(Vector3.UnitZ.ToArray(), circle.Normal.ToArray(), LinearAlgebra.AbsFloat(1e-7f));
            Assert.AreEqual(1f, circle.Radius);
            
        }

        [TestMethod()]
        public void ConvertFromToTest()
        {
            var circle = new Circle(Vector3.UnitX, Vector3.UnitY, 1);

            circle = circle.ConvertFromTo(UnitSystem.IPS, UnitSystem.MMKS);

            CollectionAssert.AreEqual((25.4f*Vector3.UnitX).ToArray(), circle.Center.ToArray(), LinearAlgebra.AbsFloat(2e-6f));
            Assert.AreEqual(Vector3.UnitY, circle.Normal);
            Assert.AreEqual(25.4f, circle.Radius, 2e-6f);
        }
    }
}