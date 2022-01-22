using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics
{
    public static class Constants
    {
        public static Random RandomNumberGenerator { get; } = new Random();

        const float pi = 3.1415926535897932384626433832795f;
        const float deg = pi/180f;
        const float rad = 1f / deg;

        public static float Pi(this int x) => x * pi;
        public static float Pi(this float x) => x * pi;
        public static float InvPi(this int x) => x / pi;
        public static float InvPi(this float x) => x / pi;
        public static float Deg(this int x) => x * deg;
        public static float Deg(this float x) => x * deg;
        public static float Rad(this int x) => x * rad;
        public static float Rad(this float x) => x * rad;
        public static float Rpm(this int x) => (2 * pi * x) / 60;
        public static float Rpm(this float x) => (2 * pi * x) / 60;
        public static float Mm(this int x) => x * 0.001f;
        public static float Mm(this float x) => x * 0.001f;

        public static float Random(float minValue = 0, float maxValue = 1)
            => minValue + (maxValue - minValue) * (float)RandomNumberGenerator.NextDouble();
    }
}
