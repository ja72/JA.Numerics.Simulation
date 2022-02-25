using Microsoft.VisualStudio.TestTools.UnitTesting;
using JA.Numerics.Simulation;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics.Simulation
{
    [TestClass()]
    public class UnitTests
    {
        [TestMethod()]
        public void FactorTest()
        {
            // Check some common conversions
            Assert.AreEqual(0.001f, Unit.Length.Factor(UnitSystem.MMKS));
            Assert.AreEqual(0.0254f, Unit.Length.Factor(UnitSystem.IPS));
            Assert.AreEqual(12*0.0254f, Unit.Length.Factor(UnitSystem.FPS));
            Assert.AreEqual(4.4482216f, Unit.Force.Factor(UnitSystem.IPS));
            Assert.AreEqual(0.4535924f, Unit.Mass.Factor(UnitSystem.IPS));
            Assert.AreEqual(1/1.8f, Unit.Temperature.Factor(UnitSystem.IPS));
        }

        [TestMethod()]
        public void ConvertTest()
        {
            const float tol = 0.001f;

            // Check some common conversions
            Assert.AreEqual(0.00064516f, Unit.Area.Convert(UnitSystem.IPS, UnitSystem.SI), tol);
            Assert.AreEqual(645.16f, Unit.Area.Convert(UnitSystem.IPS, UnitSystem.MMKS), tol);
            Assert.AreEqual(0.175f, (Unit.Force/Unit.Length).Convert(UnitSystem.IPS, UnitSystem.MMKS), tol);
        }
        [TestMethod()]
        public void GetBaseTest()
        {
            Assert.AreEqual(UnitType.Length, Unit.Length.GetBase());
            Assert.AreEqual(UnitType.Length, Unit.Area.GetBase());
            Assert.AreEqual(UnitType.Length, Unit.Volume.GetBase());
            Assert.AreEqual(UnitType.Mass, Unit.Mass.GetBase());
            Assert.AreEqual(UnitType.Force, Unit.Force.GetBase());
            Assert.AreEqual(UnitType.Temperature, Unit.Temperature.GetBase());
            Assert.AreEqual(UnitType.Temperature, Unit.PerTemperature.GetBase());
            Assert.AreEqual(UnitType.None, Unit.Momentum.GetBase());
        }

        [TestMethod()]
        public void BaseTest()
        {
            Assert.AreEqual(Unit.Length, Unit.Base(UnitType.Length));
            Assert.AreEqual(Unit.Time, Unit.Base(UnitType.Time));
            Assert.AreEqual(Unit.Force, Unit.Base(UnitType.Force));
            Assert.AreEqual(Unit.Mass, Unit.Base(UnitType.Mass));

            Assert.AreNotEqual(Unit.PerTemperature, Unit.Base(UnitType.Temperature));
        }

        [TestMethod()]
        public void CombineTest()
        {
            Assert.AreEqual( Unit.Speed, Unit.Length/ Unit.Time);
            Assert.AreEqual( Unit.Power, Unit.Force * Unit.Speed);
            Assert.AreEqual(Unit.Power, Unit.Work/Unit.Time);
        }

        [TestMethod()]
        public void DerivedTest()
        {
            var cm = 10*Unit.Length;

            Assert.AreEqual(1/100f, cm.Factor(UnitSystem.MMKS), 0.0001f);
        }

        [TestMethod()]
        public void RaiseTest()
        {
            Assert.AreEqual(Unit.None, Unit.Length ^ 0);
            Assert.AreEqual(Unit.Length, Unit.Length ^ 1);
            Assert.AreEqual(Unit.Area, Unit.Length ^ 2);

            Assert.AreEqual(100*Unit.Area, (10*Unit.Length)^2);
        }


    }
}