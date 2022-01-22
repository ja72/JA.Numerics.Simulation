using System;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace JA.Numerics
{
    public delegate TData DataFunction<out TData>(int index);
    public delegate TData MatrixFunction<out TData>(int row, int col);

    public class Matrix<TData> : IEquatable<Matrix<TData>> where TData:IEquatable<TData>
    {
        protected internal TData[] Data { get; }

        #region Factory
        public Matrix(MatrixShape shape, TData[] data)
        {
            Shape = shape;
            Data = data;
        }
        public Matrix(Matrix<TData> copy)
        {
            Shape = copy.Shape;
            Data = copy.Data.Clone() as TData[];
        }
        public Matrix(MatrixStructure structure, int size)
        {
            Shape = new MatrixShape(structure, size);
            Data = new TData[Shape.DataCount];
        }
        public Matrix(MatrixStructure structure, int size, DataFunction<TData> init)
            : this(structure, size)
        {
            for (int index = 0; index < Data.Length; index++)
            {
                Data[index] = init(index);
            }
        }
        public Matrix(MatrixStructure structure, int size, MatrixFunction<TData> init)
            : this(structure, size)
        {
            for (int i = 0; i < size; i++)
            {
                for (int j = 0; j < size; j++)
                {
                    var idx = Shape.GetDataIndex(i, j);
                    if (idx >= 0)
                    {
                        Data[idx] = init(i, j);
                    }
                }
            }
        }
        public Matrix(MatrixStructure structure, int size, TData[] data)
        {
            Shape = new MatrixShape(structure, size);
            Data = data;
        }
        public Matrix(TData[][] matrix)
            : this(MatrixStructure.Dense, matrix)
        { }
        protected Matrix(MatrixStructure structure, TData[][] matrix)
            : this(structure, matrix.Length)
        {
            for (int i = 0; i < matrix.Length; i++)
            {
                var row = matrix[i];
                for (int j = 0; j < row.Length; j++)
                {
                    this[i, j] = row[j];
                }
            }
        }
        #endregion

        #region Properties
        public MatrixShape Shape { get; }
        public MatrixStructure Structure { get => Shape.Structure; }
        public int Size { get => Shape.Size; }
        public int Count { get => Data.Length; }
        public bool IsSquare { get => Shape.IsSquare; }
        public bool IsVector { get => Shape.IsVector; }
        public bool IsZero { get => Shape.IsZero; }
        public bool IsDiagonal { get => Shape.IsDiagonal; }
        public bool IsFull { get => Shape.IsFull; }
        public bool IsSymmetric { get => Shape.IsSymmetric; }

        protected internal ref TData this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => ref Data[index];
        }
        public virtual TData this[int row, int column]
        {

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                var idx = Shape.GetDataIndex(row, column);
                return idx < 0 ? default : Data[idx];
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                var idx = Shape.GetDataIndex(row, column);
                if (idx >= 0)
                {
                    Data[idx] = value;
                }
            }
        }

        #endregion

        #region Methods
        protected TData[] GetRow(int row, TData zero = default)
        {
            return Shape.GetRowIndex(row).Select((idx) => idx >= 0 ? Data[idx] : zero).ToArray();
        }
        protected TData[] GetColumn(int column, TData zero = default)
        {
            return Shape.GetColumnIndex(column).Select((idx) => idx >= 0 ? Data[idx] : zero).ToArray();
        }
        protected TData[] GetDiagonal(TData zero = default)
        {
            return Shape.GetDiagonalIndex().Select((idx) => idx >= 0 ? Data[idx] : zero).ToArray();
        }

        protected TData[][] ToJagged(bool transpose = false, TData zero = default)
        {
            var matrix = new TData[Size][];
            for (int row = 0; row < Size; row++)
            {
                matrix[row] = transpose ? GetColumn(row, zero) : GetRow(row, zero);
            }
            return matrix;
        }
        protected TData[][] InnerTranspose()
        {
            var matrix = new TData[Size][];
            for (int j = 0; j < matrix.Length; j++)
            {
                var tmp = new TData[Size];
                for (int i = 0; i < tmp.Length; i++)
                {
                    tmp[i] = this[i, j];
                }
                matrix[j] = tmp;
            }
            return matrix;
        }

        #endregion

        #region IEquatable Members

        /// <summary>
        /// Equality overrides from <see cref="System.Object"/>
        /// </summary>
        /// <param name="obj">The object to compare this with</param>
        /// <returns>False if object is a different type, otherwise it calls <code>Equals(Matrix)</code></returns>
        public override bool Equals(object obj)
        {
            if (obj is Matrix<TData> matrix)
            {
                return Equals(matrix);
            }
            return false;
        }

        /// <summary>
        /// Checks for equality among <see cref="Matrix"/> classes
        /// </summary>
        /// <returns>True if equal</returns>
        public bool Equals(Matrix<TData> other)
        {
            return Shape.Equals(other.Shape)
                && LinearAlgebra.Equals(Data, other.Data);
        }
        /// <summary>
        /// Calculates the hash code for the <see cref="Matrix"/>
        /// </summary>
        /// <returns>The int hash value</returns>
        public override int GetHashCode()
        {
            unchecked
            {
                int hc = -1817952719;
                hc = (-1521134295) * hc + Shape.GetHashCode();
                hc = (-1521134295) * hc + LinearAlgebra.GetHashCode(Data);
                return hc;
            }
        }

        #endregion

    }

    public sealed class Matrix : Matrix<float>, IEquatable<Matrix>
    {
        #region Factory
        public Matrix(MatrixStructure structure, float[] data)
            : base(MatrixShape.GuessFromDataSize(structure, data.Length), data) { }
        public Matrix(MatrixStructure structure, int size)
            : base(structure, size)
        { }
        public Matrix(MatrixStructure structure, int size, DataFunction<float> init)
            : base(structure, size, init)
        { }
        public Matrix(MatrixStructure structure, int size, MatrixFunction<float> init)
            : base(structure, size, init)
        { }
        public Matrix(MatrixStructure structure, int size, float[] data)
            : base(structure, size, data)
        { }
        public Matrix(float[][] matrix)
            : this(MatrixStructure.Dense, matrix)
        { }
        Matrix(MatrixStructure structure, float[][] matrix)
            : base(structure, matrix)
        { }
        public static readonly Matrix Empty = new Matrix(MatrixStructure.Zero, 0);
        public static Matrix Zero(int size)
        {
            return new Matrix(MatrixStructure.Zero, size);
        }
        public static Matrix Identity(int size) => Diagonal(size, 1);
        public static Matrix Diagonal(int size, float value)
        {
            return new Matrix(MatrixStructure.Diagonal, size, Enumerable.Repeat(value, size).ToArray());
        }
        public static Matrix Diagonal(params float[] values)
        {
            return new Matrix(MatrixStructure.Diagonal, values.Length, values);
        }
        public static Matrix Symmetric(params float[] values)
        => new Matrix(MatrixStructure.Symmetric, values);

        public static Matrix Random(int size, float minValue = 0, float maxValue = 1)
        {
            return Dense(size, LinearAlgebra.RandomArray(size * size, minValue, maxValue));
        }
        public static Matrix Random(MatrixStructure structure, int size, float minValue = 0, float maxValue = 1)
        {
            var dataCount = MatrixShape.GetDataSize(structure, size, size);
            return new Matrix(structure, size, LinearAlgebra.RandomArray(dataCount, minValue, maxValue));
        }
        public static Matrix Dense(params float[] values)
        => new Matrix(MatrixStructure.Dense, values);
        public static Matrix Dense(int size, params float[] values)
        => new Matrix(MatrixStructure.Dense, size, values);

        public static implicit operator Matrix(float scalar) => Diagonal(scalar);

        public static implicit operator Matrix(float[][] matrix) => FromJagged(matrix);
        public static Matrix FromJagged(float[][] matrix)
        {
            for (int s_index = 0; s_index < MatrixShape.StructureList.Length; s_index++)
            {
                MatrixStructure struc = MatrixShape.StructureList[s_index];
                if (CheckStructure(struc, matrix))
                {
                    return new Matrix(struc, matrix);
                }
            }
            throw new ArgumentException("Invalid Matrix.");
        }
        #endregion

        #region Conversions
        public bool CheckZeros { get => Data.All((x) => Math.Abs(x) < LinearAlgebra.ZeroTolerance); }
        static bool CheckStructure(MatrixStructure structure, float[][] matrix)
        {
            if (structure == MatrixStructure.Dense) return true;
            var size = matrix.Length;
            var shape = new MatrixShape(structure, size);

            for (int row = 0; row < size; row++)
            {
                var dataRow = matrix[row];
                for (int col = 0; col < size; col++)
                {
                    var idx = shape.GetDataIndex(row, col);
                    if (idx < 0 && Math.Abs(dataRow[col]) > LinearAlgebra.ZeroTolerance) return false;
                }
                for (int col = row + 1; col < size; col++)
                {
                    var upper = dataRow[col];
                    var lower = matrix[col][row];
                    switch (structure)
                    {
                        case MatrixStructure.Symmetric:
                            if (Math.Abs(upper - lower) >= LinearAlgebra.ZeroTolerance) return false;
                            break;
                        case MatrixStructure.Zero:
                        case MatrixStructure.Diagonal:
                        case MatrixStructure.Dense:
                        default:
                            break;
                    }
                }
            }
            return true;
        }
        public bool Is(MatrixStructure structure)
        {
            return CheckStructure(structure, ToJagged());
        }
        public Matrix As(MatrixStructure structure)
        {
            return new Matrix(structure, ToJagged());
        }
        public Matrix ToDense() => new Matrix(MatrixStructure.Dense, ToJagged());
        public Matrix ToSymmetric() => new Matrix(MatrixStructure.Symmetric, ToJagged());
        public Matrix ToDiagonal() => new Matrix(MatrixStructure.Diagonal, ToJagged());
        #endregion        

        #region Algebra

        public float Min() => Data.Min();
        public float Max() => Data.Max();
        public float MinAbs() => Data.Where((x) => x != 0).Min((x) => Math.Abs(x));
        public float MaxAbs() => Data.Max((x) => Math.Abs(x));

        public Matrix Transpose()
        {
            MatrixStructure target;
            switch (Shape.Structure)
            {
                case MatrixStructure.Zero:
                case MatrixStructure.Diagonal:
                case MatrixStructure.Symmetric:
                    return this;
                case MatrixStructure.Dense:
                    target = MatrixStructure.Dense;
                    break;
                default:
                    throw new NotSupportedException($"{Structure} matrix not supported.");
            }

            var matrix = InnerTranspose();
            return new Matrix(target, matrix);

        }
        public static Matrix Negate(Matrix a)
        {
            if (a.IsZero)
            {
                return a;
            }
            var data = new float[a.Count];
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = -a.Data[i];
            }
            return new Matrix(a.Structure, a.Size, data);
        }

        public static Matrix Add(Matrix a, Matrix b)
        {
            if (a.Size != b.Size)
            {
                throw new ArgumentOutOfRangeException(nameof(b));
            }
            if (a.IsZero)
            {
                return b;
            }
            if (b.IsZero)
            {
                return a;
            }
            if (a.Structure == b.Structure)
            {
                var data = new float[a.Count];
                for (int index = 0; index < data.Length; index++)
                {
                    data[index] = a.Data[index] + b.Data[index];
                }
                var result = new Matrix(a.Structure, a.Size, data);
                if (a.Structure == MatrixStructure.Dense)
                {
                    for (int i_struc = 0; i_struc < MatrixShape.StructureSparse.Length; i_struc++)
                    {
                        MatrixStructure struc = MatrixShape.StructureSparse[i_struc];
                        if (result.Is(struc)) return result.As(struc);
                    }
                }
                return result;
            }

            var matrix = new float[a.Size][];
            for (int i = 0; i < matrix.Length; i++)
            {
                var row = new float[a.Size];
                for (int j = 0; j < row.Length; j++)
                {
                    row[j] = a[i, j] + b[i, j];
                }
                matrix[i] = row;
            }
            return FromJagged(matrix);
        }
        public static Matrix Subtract(Matrix a, Matrix b)
        {
            if (a.Size != b.Size)
            {
                throw new ArgumentOutOfRangeException(nameof(b));
            }
            if (a.IsZero)
            {
                return Negate(b);
            }
            if (b.IsZero)
            {
                return a;
            }

            if (a.Structure == b.Structure)
            {
                var data = new float[a.Count];
                for (int index = 0; index < data.Length; index++)
                {
                    data[index] = a.Data[index] - b.Data[index];
                }
                var result = new Matrix(a.Structure, a.Size, data);
                if (result.Structure == MatrixStructure.Dense)
                {
                    for (int i_struc = 0; i_struc < MatrixShape.StructureSparse.Length; i_struc++)
                    {
                        MatrixStructure struc = MatrixShape.StructureSparse[i_struc];
                        if (result.Is(struc)) return result.As(struc);
                    }
                }
                return result;
            }

            var matrix = new float[a.Size][];
            for (int i = 0; i < matrix.Length; i++)
            {
                var row = new float[a.Size];
                for (int j = 0; j < row.Length; j++)
                {
                    row[j] = a[i, j] - b[i, j];
                }
                matrix[i] = row;
            }
            return FromJagged(matrix);
        }

        public static Matrix Scale(float factor, Matrix a)
        {
            if (a.IsZero)
            {
                return a;
            }
            if (factor == 0)
            {
                return Zero(a.Size);
            }
            var data = new float[a.Count];
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = factor * a.Data[i];
            }
            return new Matrix(a.Structure, a.Size, data);
        }

        static Matrix Product(Matrix A, Matrix B)
        {
            int n = A.Size;
            int m = B.Size;
            if (m != n)
            {
                throw new ArgumentOutOfRangeException(nameof(B));
            }
            if (A.IsZero || B.IsZero)
            {
                return Zero(n);
            }
            if (A.IsDiagonal && B.IsDiagonal)
            {
                float[] result = new float[n];
                for (int k = 0; k < n; k++)
                {
                    result[k] = A.Data[k] * B.Data[k];
                }
                return Diagonal(result);
            }
            else
            {
                float[][] result = new float[n][];
                for (int i = 0; i < n; i++)
                {
                    var row = new float[B.Size];
                    for (int j = 0; j < row.Length; j++)
                    {
                        float sum = 0;
                        for (int k = 0; k < m; k++)
                        {
                            sum += A[i, k] * B[k, j];
                        }
                        row[j] = sum;
                    }
                    result[i] = row;
                }
                return FromJagged(result);
            }
        }

        public Matrix Inverse()
        {
            switch (Structure)
            {
                case MatrixStructure.Diagonal:
                    return new Matrix(Structure, Size, (i)=>1/Data[i]);
                case MatrixStructure.Symmetric:
                case MatrixStructure.Dense:
                    return JaggedMatrixLU.MatrixInverse(ToJagged());
                case MatrixStructure.Zero:
                default:
                    throw new NotSupportedException($"{Structure} not supported.");
            }
        }
        public Vector Solve(Vector b)
        {
            if (Size != b.Size)
            {
                throw new ArgumentOutOfRangeException(nameof(b));
            }
            float[] x = new float[Size];
            switch (Structure)
            {
                case MatrixStructure.Diagonal:
                    {
                        for (int i = 0; i < Size; i++)
                        {
                            x[i] = b[i] / Data[i];
                        }
                        break;
                    }
                case MatrixStructure.Symmetric:
                case MatrixStructure.Dense:
                    x = JaggedMatrixLU.SystemSolve(ToJagged(), b.Data);
                    break;
                case MatrixStructure.Zero:
                default:
                    throw new NotSupportedException(Structure.ToString());
            }
            return Vector.FromArray(x);
        }
        public Matrix Solve(Matrix B)
        {
            int n = B.Size;
            if (Size != n)
            {
                throw new ArgumentOutOfRangeException(nameof(B));
            }
            if (Shape.IsSingular)
            {
                throw new NotSupportedException("Matrix is singular.");
            }
            if (B.IsZero)
            {
                return Zero(n);
            }
            if (Structure == MatrixStructure.Diagonal && B.Structure == MatrixStructure.Diagonal)
            {
                float[] result = new float[Size];
                for (int i = 0; i < result.Length; i++)
                {
                    result[i] = B[i] / Data[i];
                }
                return Diagonal(result);
            }
            return FromJagged(JaggedMatrixLU.SystemSolve(ToJagged(), B.ToJagged()));
        }
        #endregion

        #region Operators
        public static Matrix operator +(float lhs, Matrix rhs) { return Add(Diagonal(rhs.Size, lhs), rhs); }
        public static Matrix operator +(Matrix lhs, float rhs) { return Add(lhs, Diagonal(lhs.Size, rhs)); }
        public static Matrix operator +(Matrix lhs, Matrix rhs) { return Add(lhs, rhs); }
        public static Matrix operator -(float lhs, Matrix rhs) { return Subtract(Diagonal(rhs.Size, lhs), rhs); }
        public static Matrix operator -(Matrix lhs, float rhs) { return Subtract(lhs, Diagonal(lhs.Size, rhs)); }
        public static Matrix operator -(Matrix lhs, Matrix rhs) { return Subtract(lhs, rhs); }
        public static Matrix operator +(Matrix rhs) { return Scale(1, rhs); }
        public static Matrix operator -(Matrix rhs) { return Scale(-1, rhs); }
        public static Matrix operator *(float lhs, Matrix rhs) { return Scale(lhs, rhs); }
        public static Matrix operator *(Matrix lhs, float rhs) { return Scale(rhs, lhs); }
        public static Matrix operator *(Matrix lhs, Matrix rhs) { return Product(lhs, rhs); }
        public static Matrix operator /(Matrix lhs, float rhs) { return Scale(1 / rhs, lhs); }
        public static Matrix operator ~(Matrix rhs) { return rhs.Transpose(); }

        public static Matrix operator !(Matrix rhs) { return rhs.Inverse(); }
        public static Vector operator /(Vector lhs, Matrix rhs)
        {
            return rhs.Solve(lhs);
        }
        public static Matrix operator /(Matrix lhs, Matrix rhs)
        {
            return rhs.Solve(lhs);
        }
        #endregion

        public bool Equals(Matrix matrix) => base.Equals(matrix);
    }
}
