using System.Drawing;
using System.Windows.Forms;

namespace JA.Numerics.UI
{
    using System.ComponentModel;

    [TypeConverter(typeof(ExpandableObjectConverter))]
    public class MouseControl
    {
        public Point DownPos { get; set; }
        public Point UpPos { get; set; }
        public Point CurrentPos { get; set; }
        public MouseButtons Buttons { get; set; }

        public (int dx, int dy) CurrentDelta => (CurrentPos.X - DownPos.X, CurrentPos.Y - DownPos.Y);
        public (int dx, int dy) DragDelta => (UpPos.X - DownPos.X, UpPos.Y - DownPos.Y);
    }
}
