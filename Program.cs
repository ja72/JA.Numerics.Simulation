using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace JA
{
    static class Program
    {
        [STAThread()]
        static void Main(string[] args)
        {
            Console.OutputEncoding = Encoding.Unicode;

            //Planar.World2.Demo();

            //Spatial.World3.Demo();

            Application.Run(new UI.Render3DForm());
        }
    }
}
