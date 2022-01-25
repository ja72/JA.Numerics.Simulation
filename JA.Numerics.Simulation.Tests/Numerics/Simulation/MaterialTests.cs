using Microsoft.VisualStudio.TestTools.UnitTesting;
using JA.Numerics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;

namespace JA.Numerics.Simulation
{
    [TestClass()]
    public class MaterialTests
    {
        [TestMethod()]
        public void MaterialTest()
        {
            Material al_si = MaterialSpec.Aluminum;

            //Material: Aluminum
            //
            //Units  Density    Sym         Elastic     Sym       CTE       Sym     
            //----------------------------------------------------------------------
            //SI     2690       kg/m^3      6.895e+10   Pa        2.394e-05 1/°C    
            //MMKS   2.69e-06   kg/mm^3     6.895e+04   MPa       2.394e-05 1/°C    
            //IPS    0.09718    lb/in^3     1e+07       psi       1.33e-05  1/°F    
            //FPS    167.9      lb/ft^3     1.44e+09    psf       1.33e-05  1/°F    

            Material al_ips = al_si.ConvertTo(UnitSystem.IPS);
            Material al_mmks = new Material(
                UnitSystem.MMKS,
                2.69e-6f,
                68950f,
                0.33f,
                23.94e-6f, MaterialSpec.Aluminum);

            var al_si_mmks = al_si.ConvertTo(UnitSystem.MMKS);
            var al_ips_mmks = al_ips.ConvertTo(UnitSystem.MMKS);

            Assert.AreEqual(al_mmks.Density, al_si_mmks.Density, 1e-9f);
            Assert.AreEqual(al_mmks.Elastic, al_si_mmks.Elastic, 1e-1f);
            Assert.AreEqual(al_mmks.Poissons, al_si_mmks.Poissons);
            Assert.AreEqual(al_mmks.CTE, al_si_mmks.CTE, 1e-7f);

            Assert.AreEqual(al_mmks.Density, al_ips_mmks.Density, 1e-9f);
            Assert.AreEqual(al_mmks.Elastic, al_ips_mmks.Elastic, 1e-1f);
            Assert.AreEqual(al_mmks.Poissons, al_ips_mmks.Poissons);
            Assert.AreEqual(al_mmks.CTE, al_ips_mmks.CTE, 1e-7f);
        }
    }
}