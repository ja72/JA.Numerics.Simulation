using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial
{

    [TestClass()]
    public class PoseTests
    {
        [TestMethod()]
        public void PoseTest()
        {
            var r = LinearAlgebra.RandomVector(-1f, 5f);
            var q = Quaternion.CreateFromYawPitchRoll(
                Constants.Deg(LinearAlgebra.Random(-90, 90)),
                Constants.Deg(LinearAlgebra.Random(-90, 90)),
                Constants.Deg(LinearAlgebra.Random(-90, 90)));

            var P = new Pose(r, q);

            Assert.AreEqual(r, P.Position);
            Assert.AreEqual(q, P.Orientation);

            P = r;
            Assert.AreEqual(r, P.Position);
            Assert.AreEqual(Quaternion.Identity, P.Orientation);

            P = q;
            Assert.AreEqual(Vector3.Zero, P.Position);
            Assert.AreEqual(q, P.Orientation);
        }


        [TestMethod()]
        public void FromLocalTest()
        {
            var r = LinearAlgebra.RandomVector(-1f, 5f);
            var q = Quaternion.CreateFromYawPitchRoll(
                Constants.Deg(LinearAlgebra.Random(-90, 90)),
                Constants.Deg(LinearAlgebra.Random(-90, 90)),
                Constants.Deg(LinearAlgebra.Random(-90, 90)));

            var P = new Pose(r, q);

            var g = LinearAlgebra.RandomVector(-1f, 5f);
            var b = Quaternion.CreateFromYawPitchRoll(
                Constants.Deg(LinearAlgebra.Random(-90, 90)),
                Constants.Deg(LinearAlgebra.Random(-90, 90)),
                Constants.Deg(LinearAlgebra.Random(-90, 90)));

            var R = new Pose(g, b);

            var K = P * R;

            Assert.AreEqual(K.Position, r + g.Rotate(q));
            Assert.AreEqual(K.Orientation, q * b);

            P = Pose.AlongX(2f) * Pose.AboutZ(Constants.Deg(-90)) * Pose.AlongX(1f);
            g = new Vector3(2, -1, 0);
            Assert.AreEqual(g.X, P.Position.X, 1e-4f);
            Assert.AreEqual(g.Y, P.Position.Y, 1e-4f);
            Assert.AreEqual(g.Z, P.Position.Z, 1e-4f);
        }

    }
}