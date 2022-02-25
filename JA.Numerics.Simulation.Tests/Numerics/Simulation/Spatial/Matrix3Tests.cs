using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace JA.Numerics.Simulation.Spatial
{
    /// <summary>
    /// Summary description for Matrix3
    /// </summary>
    [TestClass]
    public class Matrix3Tests
    {
        public Matrix3Tests()
        {
        }

        private TestContext testContextInstance;

        /// <summary>
        ///Gets or sets the test context which provides
        ///information about and functionality for the current test run.
        ///</summary>
        public TestContext TestContext
        {
            get
            {
                return testContextInstance;
            }
            set
            {
                testContextInstance = value;
            }
        }

        #region Additional test attributes
        //
        // You can use the following additional attributes as you write your tests:
        //
        // Use ClassInitialize to run code before running the first test in the class
        // [ClassInitialize()]
        // public static void MyClassInitialize(TestContext testContext) { }
        //
        // Use ClassCleanup to run code after all tests in a class have run
        // [ClassCleanup()]
        // public static void MyClassCleanup() { }
        //
        // Use TestInitialize to run code before running each test 
        // [TestInitialize()]
        // public void MyTestInitialize() { }
        //
        // Use TestCleanup to run code after each test has run
        // [TestCleanup()]
        // public void MyTestCleanup() { }
        //
        #endregion

        [TestMethod]
        public void CreateRotationTest()
        {
            for (int i = 0; i < 10; i++)
            {
                float x = 0.2f*i;
                Assert.AreEqual(
                    Matrix3.CreateRotationX(x),
                    Matrix3.CreateRotation(Vector3.UnitX, x));
                Assert.AreEqual(
                    Matrix3.CreateRotationY(x),
                    Matrix3.CreateRotation(Vector3.UnitY, x));
                Assert.AreEqual(
                    Matrix3.CreateRotationZ(x),
                    Matrix3.CreateRotation(Vector3.UnitZ, x));
            }
            var q = Quaternion.CreateFromYawPitchRoll(
                Constants.Deg(LinearAlgebra.Random(-90, 90)),
                Constants.Deg(LinearAlgebra.Random(-90, 90)),
                Constants.Deg(LinearAlgebra.Random(-90, 90)));

            var R = Matrix3.CreateRotation(q);

            var m4 = Matrix4x4.CreateFromQuaternion(q);
            var P = m4.Rotation();

            var Z = R - P;

            CollectionAssert.AreEqual(R.ToArray(), P.ToArray(), LinearAlgebra.AbsFloat(1e-4f) );
        }

        [TestMethod]
        public void MatrixAlgebraTest()
        {
            var A = Matrix3.Random(-1f, 5f);
            Matrix3 As = A.Symmetric(), Ak = A.SkewSymmetric();

            AbsFloatComparer approx = LinearAlgebra.AbsFloat(1e-4f);

            CollectionAssert.AreEqual(A.ToArray(), (As + Ak).ToArray(), approx);

            var C = Matrix3.Identity * A;
            CollectionAssert.AreEqual(A.ToArray(), C.ToArray(), approx);

            var B = Matrix3.Random(-1f, 5f);
            C = A * B;
            var B2 = A.Solve(C);
            CollectionAssert.AreEqual(B.ToArray(), B2.ToArray(), approx);
        }
    }
}
