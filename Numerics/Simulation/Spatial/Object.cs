using System;
using System.ComponentModel;
using System.Drawing;
using JA.UI;

namespace JA.Numerics.Simulation.Spatial
{
    [TypeConverter(typeof(ExpandableObjectConverter))]
    public abstract class Object : IHasUnits<Object>
    {
        public abstract void Render(Graphics g, Camera camera, Pose pose);
        public abstract override string ToString();
        protected abstract Object InnerConvertTo(UnitSystem target);

        protected Object(World3 world)
        {
            World=world??throw new ArgumentNullException(nameof(world));
            Units = World.Units;
        }
        protected Object(Object copy)
        {
            World = copy.World;
            Units = copy.Units;
        }
        protected Object(Object copy, UnitSystem target)
        {
            World = copy.World;
            Units = target;
        }

        [Browsable(false)]
        public World3 World { get; }
        public UnitSystem Units { get; }

        Object IHasUnits<Object>.ConvertTo(UnitSystem target)
            => InnerConvertTo(target);
    }
}