using System;

namespace JA.Numerics.Simulation
{

    public enum MaterialSpec
    {
        Custom,
        Aluminum,
        Steel,
        CastIron,
    }
    public sealed class Material :
        ICanChangeUnits<Material>,
        IEquatable<Material>
    {
        public Material(UnitSystem units, float density, float elastic, float poissons, float cte, MaterialSpec spec = MaterialSpec.Custom)
        {
            Spec = spec;
            Units = units;
            Density = density;
            Elastic = elastic;
            Poissons = poissons;
            CTE = cte;
        }
        Material(MaterialSpec spec)
        {
            Units = UnitSystem.SI;
            Spec = spec;
            switch (spec)
            {
                case MaterialSpec.Aluminum:
                    Density = 2.69e3f;
                    Elastic = 68950e6f;
                    Poissons = 0.33f;
                    CTE = 23.94e-6f;
                    break;
                case MaterialSpec.Steel:
                    Density = 7.68e3f;
                    Elastic = 206800e6f;
                    Poissons = 0.3f;
                    CTE = 11.7e-6f;
                    break;
                case MaterialSpec.CastIron:
                    Density = 7.40e3f;
                    Elastic = 176500f;
                    Poissons = 0.27f;
                    CTE = 5.8e-6f;
                    break;
                case MaterialSpec.Custom:
                default:
                    throw new NotImplementedException();
            }
        }
        public static implicit operator Material(MaterialSpec spec) => Library(spec);

        public static Material Library(MaterialSpec spec) => new Material(spec);

        internal static Material Test(UnitSystem units) => new Material(units, 1, 1, 1, 1);

        public UnitSystem Units { get; }
        public MaterialSpec Spec { get; }
        public float Density { get; }
        public float Elastic { get; }
        public float Poissons { get;  }
        public float CTE { get;  }

        public Material ConvertTo(UnitSystem target)
        {
            if (Units != target)
            {
                return new Material(target,
                    Density * Unit.Density.Convert(Units, target),
                    Elastic * Unit.Pressure.Convert(Units, target),
                    Poissons,
                    CTE * Unit.PerTemperature.Convert(Units, target),
                    Spec);
            }
            return this;
        }


        #region IEquatable Members

        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Material)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Material item)
            {
                return Equals(item);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among <see cref="Material"/> classes
        /// </summary>
        /// <returns>True if equal</returns>
        public bool Equals(Material other)
        {
            if (Spec == MaterialSpec.Custom || other.Spec == MaterialSpec.Custom)
            {
                other = other.ConvertTo(Units);

                return Density.Equals(other.Density)
                    && Elastic.Equals(other.Elastic)
                    && Poissons.Equals(other.Poissons)
                    && CTE.Equals(other.CTE);
            }
            else
            {
                return Spec == other.Spec;
            }
        }
        /// <summary>
        /// Calculates the hash code for the <see cref="Material"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc = (-1521134295) * hc + Units.GetHashCode();
                hc = (-1521134295) * hc + Density.GetHashCode();
                hc = (-1521134295) * hc + Elastic.GetHashCode();
                hc = (-1521134295) * hc + Poissons.GetHashCode();
                hc = (-1521134295) * hc + CTE.GetHashCode();
                return hc;
            }
        }

        #endregion

    }

}
