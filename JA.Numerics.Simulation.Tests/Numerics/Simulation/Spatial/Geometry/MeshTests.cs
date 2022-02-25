using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Numerics;

namespace JA.Numerics.Simulation.Spatial
{
    [TestClass()]
    public class MeshTests
    {
        [TestMethod()]
        public void GetMmoiTest()
        {
            var O = Vector3.Zero;
            var A = new Vector3(40, 0, 0);
            var B = new Vector3(0, 25, 0);
            var C = new Vector3(0, 0, 8);
            var mesh = new Geometry.Mesh(UnitSystem.MMKS);
            mesh.AddFace(Color.Green, A, B, C);
            mesh.AddFace(Color.Green, O, C, B);
            mesh.AddFace(Color.Green, O, B, A);
            mesh.AddFace(Color.Green, O, A, C);
            float mass = 0.0133333333f;

            // Values derived from CAD
            var loose = LinearAlgebra.AbsFloat(1e-4f);
            var rb = mesh.GetMmoiFromMass(mass);
            CollectionAssert.AreEqual(new[] { 9.999999f, 6.25f, 2f }, rb.CG.ToArray(), loose);

            CollectionAssert.AreEqual(new[] { 0.3445f, 0.1666f, 0.05333f }, rb.MMoi.Row1.ToArray(), loose);
            CollectionAssert.AreEqual(new[] { 0.1666f, 0.8320f, 0.03333f }, rb.MMoi.Row2.ToArray(), loose);
            CollectionAssert.AreEqual(new[] { 0.05333f, 0.03333f, 1.1125f }, rb.MMoi.Row3.ToArray(), loose);

            var tight = LinearAlgebra.AbsFloat(1e-6f);
            var mmoi = rb.GetPrincipalMmoi(out var cm);
            CollectionAssert.AreEqual(new[] { 9.999999f, 6.25f, 2f }, cm.Position.ToArray(), tight);
            CollectionAssert.AreEqual(new[] { 0.2909014f, 0.8741418f, 1.123957f }, mmoi.ToArray(), loose);

            var R = Matrix3.CreateRotation(cm.Orientation);
            Vector3 axis_1 = R.Column1, axis_2 = R.Column2, axis_3 = R.Column3;

            CollectionAssert.AreEqual(new[] { -0.9465475f, 0.268339f, -0.1790036f }, axis_1.ToArray(), loose);
            CollectionAssert.AreEqual(new[] { 0.2970076f, 0.9415225f, -0.1591283f }, axis_2.ToArray(), loose);
            CollectionAssert.AreEqual(new[] { 0.1258356f, -0.203788f, -0.9708945f }, axis_3.ToArray(), loose);
        }
    }
}