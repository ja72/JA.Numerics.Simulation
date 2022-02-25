using System;
using System.ComponentModel;
using System.Drawing;
using JA.UI;

namespace JA.Numerics.Simulation.Spatial
{
    /// <summary>
    /// Simulation Object base class that relates to a <see cref="World3"/> and implements <see cref="ICanConvert{Object}"/>
    /// </summary>
    /// <seealso cref="Shape" />
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public abstract class Object : ICanChangeUnits
    {
        private UnitSystem units;

        public abstract void Render(Graphics g, Camera camera, Pose pose, float sclae=1);
        public abstract override string ToString();

        protected Object(World3 world)
        {
            World=world??throw new ArgumentNullException(nameof(world));
            units = World.Units;
        }
        protected Object(Object copy)
        {
            World = copy.World;
            units = copy.Units;
        }
        protected Object(Object copy, UnitSystem target)
        {
            World = copy.World;
            units = target;
        }

        [Browsable(false)]
        public World3 World { get; }
        public UnitSystem Units
        {
            get => units;
            set
            {
                ConvertTo(value);
            }
        }
        public virtual void ConvertTo(UnitSystem target) { units = target; }
    }
}