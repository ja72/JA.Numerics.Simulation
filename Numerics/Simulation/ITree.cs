using System.Collections.Generic;

namespace JA.Numerics.Simulation
{
    public interface ITree<out TFrame>
        where TFrame : class, ITree<TFrame>
    {
        IReadOnlyList<TFrame> Children { get; }
        int Index { get; }
        bool IsRoot { get; }
        int Level { get; }
        TFrame Parent { get; }
        int ParentIndex { get; }        
    }
}