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

    public abstract class Tree<T> : 
        ITree<T> where T : Tree<T>
    {
        public T Parent { get; }
        public abstract IReadOnlyList<T> Children { get; }
        public abstract int Index { get; }
        public int ParentIndex { get => Parent != null ? Parent.Index : -1; }
        public int Level { get => IsRoot ? 0 : Parent.Level + 1; }
        public bool IsRoot { get => Parent == null; }
    }
}
