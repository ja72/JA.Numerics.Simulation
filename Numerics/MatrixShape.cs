using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;

namespace JA.Numerics
{
    public enum MatrixStructure
    {
        Zero,
        Diagonal,
        Symmetric,
        Dense,
    }

    public struct MatrixShape : IEquatable<MatrixShape>
    {
        #region Static Structure Collections
        public static readonly MatrixStructure[] StructureList = new[] {
                MatrixStructure.Zero,
                MatrixStructure.Diagonal,
                MatrixStructure.Symmetric,
                MatrixStructure.Dense,
            };
        public static readonly MatrixStructure[] StructureSparse = new[] {
                MatrixStructure.Zero,
                MatrixStructure.Diagonal,
                MatrixStructure.Symmetric,
            };
        public static readonly MatrixStructure[] StructureInvertible = new[] {
                MatrixStructure.Diagonal,
                MatrixStructure.Symmetric,
                MatrixStructure.Dense,
            };
        #endregion

        #region Factory
        public MatrixShape(MatrixStructure structure, int size)
        {
            Structure = structure;
            Size = size;
            DataCount = GetDataSize(structure, size);
        }

        public static MatrixShape Zero(int size) => new MatrixShape(MatrixStructure.Zero, size);
        public static MatrixShape Diagonal(int size) => new MatrixShape(MatrixStructure.Diagonal, size);
        public static MatrixShape Symmetric(int size) => new MatrixShape(MatrixStructure.Symmetric, size);
        public static MatrixShape Dense(int size) => new MatrixShape(MatrixStructure.Dense, size);

        public static MatrixShape GuessFromDataSize(MatrixStructure structure, int dataSize)
        {
            switch (structure)
            {
                case MatrixStructure.Zero:
                    if (dataSize == 0)
                    {
                        return new MatrixShape(structure, 0);
                    }
                    else
                        throw new NotSupportedException("Zero matrix must have zero data stored.");
                case MatrixStructure.Diagonal:
                    return new MatrixShape(structure, dataSize);
                case MatrixStructure.Symmetric:
                    return new MatrixShape(structure, (int)((Math.Sqrt(8*dataSize+1)-1)/2));
                case MatrixStructure.Dense:
                    return new MatrixShape(structure, (int)(Math.Sqrt(dataSize)));
                default:
                    throw new NotSupportedException($"{structure} matrix not supported.");
            }
        }

        #endregion

        #region Properties
        public MatrixStructure Structure { get; }
        public int Size { get; }
        public int DataCount { get; }
        public bool IsSquare { get => true; }
        public bool IsEmpty { get => Size==0 ; }
        public bool IsVector { get => Size==1; }
        public bool IsZero { get => Structure == MatrixStructure.Zero; }
        public bool IsDiagonal { get => Structure == MatrixStructure.Diagonal; }
        public bool IsFull { get => Structure == MatrixStructure.Dense; }
        public bool IsSymmetric { get => Structure == MatrixStructure.Symmetric; }
        public bool IsInvertible
        {
            get => Structure == MatrixStructure.Diagonal
            || Structure == MatrixStructure.Symmetric
            || Structure == MatrixStructure.Dense;
        }
        public bool IsSingular
        {
            get => Structure == MatrixStructure.Zero;
        }
        #endregion

        #region Indexing
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetDataIndex(int row, int column)
        {
            if (row < 0 || row >= Size || column < 0 || column >= Size)
            {
                return -1;
            }
            else
            {
                int sign;
                switch (Structure)
                {
                    case MatrixStructure.Zero:
                        sign = 0;
                        break;
                    case MatrixStructure.Diagonal:
                        sign = row == column ? 1 : 0;
                        break;
                    case MatrixStructure.Symmetric:
                    case MatrixStructure.Dense:
                        sign = 1;
                        break;
                    default:
                        throw new NotSupportedException($"{Structure} matrix not supported.");
                }
                if (sign==0) return -1;
                switch (Structure)
                {
                    case MatrixStructure.Zero:
                        return -1;
                    case MatrixStructure.Diagonal:
                        return row == column ? row : -1;
                    case MatrixStructure.Symmetric:
                        return column >= row
                        ? row * Size - row * (row - 1) / 2 + column - row
                        : column * Size - column * (column - 1) / 2 + row - column;
                    case MatrixStructure.Dense:
                        return row * Size + column;
                    default:
                        throw new NotSupportedException($"{Structure} matrix not supported.");
                }
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public (int row, int column) GetCoord(int index)
        {
            int m = Size;

            switch (Structure)
            {
                case MatrixStructure.Zero:
                    return (-1, -1);
                case MatrixStructure.Diagonal:
                    return (index, index);
                case MatrixStructure.Symmetric:
                    {
                        int row = (int)(m + 0.5 - Math.Sqrt(
                            m * (m + 1) - 2 * index + 0.25));
                        int col = index + row * (row + 1) / 2 - m * row;
                        return (row, col);
                    }
                case MatrixStructure.Dense:
                    return (index / m, index % m);
                default:
                    throw new NotSupportedException($"{Structure} matrix not supported.");
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOfFirstData(int row)
        {
            switch (Structure)
            {
                case MatrixStructure.Zero:
                    return -1;
                case MatrixStructure.Diagonal:
                    return row;
                case MatrixStructure.Symmetric:
                    return row * Size - row * (row - 1) / 2;
                case MatrixStructure.Dense:
                    return row * Size;
                default:
                    throw new NotSupportedException($"{Structure} matrix not supported.");
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int CountDataInRow(int row)
        {
            switch (Structure)
            {
                case MatrixStructure.Zero:
                    return 0;
                case MatrixStructure.Diagonal:
                    return 1;
                case MatrixStructure.Symmetric:
                    return Size - row;
                case MatrixStructure.Dense:
                    return Size;
                default:
                    throw new NotSupportedException($"{Structure} matrix not supported.");
            }
        }

        public IEnumerable<int> GetRowIndex(int row)
        {
            for (int column = 0; column < Size; column++)
            {
                yield return GetDataIndex(row, column);
            }
        }
        public IEnumerable<int> GetColumnIndex(int column)
        {
            for (int row = 0; row < Size; row++)
            {
                yield return GetDataIndex(row, column);
            }
        }
        public IEnumerable<int> GetDiagonalIndex()
        {
            for (int row = 0; row < Size; row++)
            {
                yield return GetDataIndex(row, row);
            }
        }
        #endregion

        #region Static Methods
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetDataSize(MatrixStructure structure, int size) =>
            GetDataSize(structure, size, size);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetDataSize(MatrixStructure structure, int rowCount, int columnCount)
        {
            switch (structure)
            {
                case MatrixStructure.Zero:
                    return 0;
                case MatrixStructure.Diagonal:
                    return rowCount;
                case MatrixStructure.Symmetric:
                    return rowCount * (rowCount + 1) / 2;
                case MatrixStructure.Dense:
                    return rowCount * columnCount;
                default:
                    throw new NotSupportedException($"{structure} matrix not supported.");
            }
        }
        #endregion

        #region Formatting
        public string ToIndexTable(int columnWidth = 8)
        {
            var sb = new StringBuilder();
            for (int i = 0; i < Size; i++)
            {
                sb.Append("| ");
                for (int j = 0; j < Size; j++)
                {
                    var idx = GetDataIndex(i, j);
                    if (idx>=0)
                    {
                        var item = $"[{idx}]";
                        sb.Append(item.PadLeft(columnWidth));
                    }
                    else
                    {
                        sb.Append(" ".PadLeft(columnWidth));
                    }
                    if (j < Size - 1)
                    {
                        sb.Append(' ');
                    }
                }
                sb.AppendLine(" |");
            }
            return sb.ToString();
        }

        #endregion

        #region IEquatable Members

        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(MatrixShape)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is MatrixShape item)
            {
                return Equals(item);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among <see cref="MatrixShape"/> classes
        /// </summary>
        /// <returns>True if equal</returns>
        public bool Equals(MatrixShape other)
        {
            return Structure.Equals(other.Structure)
                && Size.Equals(other.Size);
        }
        /// <summary>
        /// Calculates the hash code for the <see cref="MatrixShape"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc = (-1521134295) * hc + Structure.GetHashCode();
                hc = (-1521134295) * hc + Size.GetHashCode();
                return hc;
            }
        }
        public static bool operator ==(MatrixShape target, MatrixShape other) { return target.Equals(other); }
        public static bool operator !=(MatrixShape target, MatrixShape other) { return !target.Equals(other); }

        #endregion

    }

}