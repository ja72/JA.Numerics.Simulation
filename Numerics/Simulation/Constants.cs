using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics.Simulation
{
    public static class Constants
    {
        static readonly Random rng = new Random();
        public const float PI = 3.1415926535897932f;
        public const float DEG = PI/180f;
        public const float RAD = 1f / DEG;
        public const float DistanceTolerance = 1e-6f;
        public static Random RandomNumberGenerator => rng;
        public static float Pi(this int x) => x * PI;
        public static float Pi(this float x) => x * PI;
        public static float InvPi(this int x) => x / PI;
        public static float InvPi(this float x) => x / PI;
        public static float Deg(this int x) => x * DEG;
        public static float Deg(this float x) => x * DEG;
        public static float Rad(this int x) => x * RAD;
        public static float Rad(this float x) => x * RAD;
        public static float Rpm(this int x) => (2 * PI * x) / 60;
        public static float Rpm(this float x) => (2 * PI * x) / 60;
        public static float Mm(this int x) => x * 0.001f;
        public static float Mm(this float x) => x * 0.001f;
        public static float Sqr(this float x) => x *x;
        public static float Sqrt(this float x) => (float)Math.Sqrt(x);
        public static float Log(this float x) => (float)Math.Log(x);
        public static float Exp(this float x) => (float)Math.Exp(x);
        public static float Random(this float maxValue) => (float)(maxValue*rng.NextDouble());

    }
}
