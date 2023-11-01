using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using Unity.VisualScripting;
using UnityEngine;

public struct MatrixCustom
{
    private float[,] matrix;

    /// <summary>The amount of rows in the matrix.</summary>
    public int rows { get; private set; }
    /// <summary>The amount of columns in the matrix.</summary>
    public int columns { get; private set; }

    public MatrixCustom(int columns, int rows, params float[] values)
    {
        // Set size
        this.columns = columns;
        this.rows = rows;

        matrix = new float[columns, rows];
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                int valueIdx = r * columns + c;
                if (valueIdx < values.Length)
                    matrix[c, r] = values[valueIdx];
            }
        }
    }

    /// <summary>
    /// For constructing 6x6 matrices. (TODO: Fix this constructor)
    /// </summary>
    public MatrixCustom(float[] row0, float[] row1, float[] row2, float[] row3, float[] row4, float[] row5)
    {
        // Set size
        rows = 6;
        columns = 6;

        // Set component values
        matrix = new float[6, 6];
        for (int i = 0; i < matrix.GetLength(0); i++)
        {
            for (int j = 0; j < matrix.GetLength(1); j++)
            {
                switch (i)
                {
                    case 0:
                        matrix[i, j] = row0[j];
                        break;
                    case 1:
                        matrix[i, j] = row1[j];
                        break;
                    case 2:
                        matrix[i, j] = row2[j];
                        break;
                    case 3:
                        matrix[i, j] = row3[j];
                        break;
                    case 4:
                        matrix[i, j] = row4[j];
                        break;
                    case 5:
                        matrix[i, j] = row5[j];
                        break;
                    default:
                        throw new IndexOutOfRangeException("MatrixCustom ctor error!");
                };
            }
        }
    }

    /// <summary>
    /// Retrieves a floating-point value from the matrix.
    /// </summary>
    /// <param name="x">The column index in the matrix.</param>
    /// <param name="y">The row index in the matrix.</param>
    /// <returns>A float from this matrix.</returns>
    public float this[int x, int y]
    {
        get
        {
            if (x >= columns)
                throw new IndexOutOfRangeException($"index {x} is larger than the matrix's column count");
            if (y >= rows)
                throw new IndexOutOfRangeException($"index {y} is larger than the matrix's row count");

            return matrix[x, y];
        }
        set
        {
            if (x >= columns)
                throw new IndexOutOfRangeException($"index {x} is larger than the matrix's column count");
            if (y >= rows)
                throw new IndexOutOfRangeException($"index {y} is larger than the matrix's row count");

            matrix[x, y] = value;
        }
    }

    // public float[] GetColumn(int i)
    // {

    // }

    /// <summary>
    /// Returns the transpose of this matrix (the matrix flipped on its diagonal).
    /// </summary>
    public MatrixCustom transpose
    {
        get
        {
            MatrixCustom result = new MatrixCustom(rows, columns);
            for (int c = 0; c < columns; c++)
            {
                for (int r = 0; r < rows; r++)
                {
                    result[r, c] = this[c, r];
                }
            }

            return result;
        }
    }

    public MatrixCustom inverse
    {
        get
        {
            // TODO: Implement inverse short-hand
            return adjoint / determinant;
        }
    }

    /// <summary>
    /// Returns the determinant of a square matrix<br />
    /// NOTE: If the matrix is not square, this will output -1.<br />
    /// DEVELOPE NOTE: Not yet implemented as this becomes computationally infeasable for larger matrices!
    /// </summary>
    public float determinant
    {
        get
        {
            // TODO: Return matrix determinant
            return -1f;
        }
    }

    public MatrixCustom adjoint
    {
        get
        {
            // TODO: Return the adjoint matrix
            return cofactor.transpose;  // This is correct??? (Probably not)
        }
    }

    public MatrixCustom cofactor
    {
        get
        {
            // TODO: Return the cofactor matrix
            return new MatrixCustom();
        }
    }

    public bool isSquare { get => (rows == columns); }

    /// <summary>
    /// Computes the inverse of matrix A via a Conjugate Gradient Descent solver. (Right???)
    /// </summary>
    /// <param name="A">The symmetric, positive-definite matrix.</param>
    /// <param name="b">A column-vector.</param>
    /// <param name="x">Initial guess for column-vector solution?</param>
    /// <returns>The correct value for the column-vector, x.</returns>
    public static MatrixCustom CGDSolver(MatrixCustom A, MatrixCustom b, MatrixCustom x)
    {
        // Variables:
        //  A = Symmetric, positive-definite matrix (the global stiffness matrix)
        //  b = A column-vector (the global force vector)
        //  r = A column-vector (the global displacement vector)

        // TODO: Check eligibility of Conjugate Gradient operation (matrix must be symmetric, positive-definite)

        MatrixCustom dir = b - A * x;
        MatrixCustom residual = dir;
        
        int iter = Mathf.Min(b.rows, 20);
        for (int i = 0; i < iter; i++)
        {
            float alpha = ColumnRowMult(x.transpose, x) / ColumnRowMult(dir.transpose * A, dir);
            MatrixCustom newX = x + alpha * dir;
            MatrixCustom newResidual = residual - alpha * A * dir;
            float beta = ColumnRowMult(newResidual.transpose, newResidual) / ColumnRowMult(residual.transpose, residual);
            dir = newResidual + beta * dir;
            x = newX;
            residual = newResidual;
        }

        return x;
    }


    public static MatrixCustom operator +(MatrixCustom m1, MatrixCustom m2)
    {
        // TODO: This operator should maybe return a warning whenever the matrices don't reciprocate their size
        int minRows = Mathf.Min(m1.rows, m2.rows);
        int minColumns = Mathf.Min(m1.columns, m2.columns);

        MatrixCustom result = m1;
        for (int r = 0; r < minRows; r++)
        {
            for (int c = 0; c < minColumns; c++)
            {
                result[c, r] += m2[c, r];
            }
        }

        return result;
    }
    public static MatrixCustom operator -(MatrixCustom m1, MatrixCustom m2)
    {
        MatrixCustom result = new MatrixCustom(m1.columns, m1.rows);
        for (int r = 0; r < m1.rows; r++)
        {
            for (int c = 0; c < m1.columns; c++)
            {
                result[c, r] = m1[c, r] - m2[c, r];
            }
        }

        return result;
    }
    public static MatrixCustom operator *(MatrixCustom m1, MatrixCustom m2)
    {
        // 1. The number of columns of the 1st matrix must equal the number of rows of the 2nd matrix.
        // 2. And the result will have the same number of rows as the 1st matrix, and the same number of columns as the 2nd matrix.

        if (m1.columns != m2.rows)
            throw new ArithmeticException("The column count of the 1st matrix is not the same as the row count of the 2nd matrix. Matrix multiplication is invalid!");

        MatrixCustom result = new MatrixCustom(m2.columns, m1.rows);
        for (int r = 0; r < m1.rows; r++)   // Go through each row in the 1st matrix
        {
            for (int c = 0; c < m2.columns; c++)    // Go through each column in the 2nd matrix
            {
                // Multiply column and row together component-wise (like a dot-operation)
                // ...and then use the sum as the resulting matrix's component
                float componentResult = 0f;
                for (int i = 0; i < m2.rows; i++)
                {
                    componentResult += m1[i, r] * m2[c, i];
                }
                result[c, r] = componentResult;
            }
        }

        return result;
    }
    public static MatrixCustom operator *(MatrixCustom m1, float scalar)
    {
        // NOTE: Can the parameter be modified directly (because it's a struct) to save on memory?

        MatrixCustom result = new MatrixCustom(m1.columns, m1.rows);

        for (int c = 0; c < m1.columns; c++)
        {
            for (int r = 0; r < m1.rows; r++)
            {
                result[c, r] = m1[c, r] * scalar;
            }
        }

        return result;
    }
    public static MatrixCustom operator *(float scalar, MatrixCustom m1)
    {
        // NOTE: Can the parameter be modified directly (because it's a struct) to save on memory?

        MatrixCustom result = new MatrixCustom(m1.columns, m1.rows);

        for (int c = 0; c < m1.columns; c++)
        {
            for (int r = 0; r < m1.rows; r++)
            {
                result[c, r] = m1[c, r] * scalar;
            }
        }

        return result;
    }
    public static Vector3[] operator *(MatrixCustom m1, Vector3[] vector)
    {
        // TODO: Implement matrix multiplication for lists of 3D-vectors on the form [x,y,z,x,y,z,x...]

        // TODO: ...and then convert it back to the form [|xyz|, |xyz|, ...]

        return vector;
    }
    public static MatrixCustom operator /(MatrixCustom m1, float divider)
    {
        // NOTE: Can the parameter be modified directly (because it's a struct) to save on memory?

        MatrixCustom result = new MatrixCustom(m1.columns, m1.rows);

        for (int c = 0; c < m1.columns; c++)
        {
            for (int r = 0; r < m1.rows; r++)
            {
                result[c, r] = m1[c, r] / divider;
            }
        }

        return result;
    }

    public enum ConcMethod
    {
        /// <summary>Concatenates the columns of each matrix such that each matrix is part of a single block-row</summary>
        MakeRow,
        /// <summary>Concatenates the rows of each matrix such that each matrix is part of a single block-column</summary>
        MakeColumn
    }
    public static MatrixCustom Concatenate(ConcMethod method, MatrixCustom m1, MatrixCustom m2)
    {
        MatrixCustom result;
        switch (method)
        {
            case ConcMethod.MakeRow:
                // TODO: Validate matrix dimensions for this operation.
                result = new MatrixCustom(m1.columns + m2.columns, m1.rows);
                for (int c = 0; c < m1.columns; c++)    // Set m1-sourced values
                {
                    for (int r = 0; r < m1.rows; r++)
                    {
                        result[c, r] = m1[c, r];
                    }
                }
                for (int c = 0; c < m2.columns; c++)    // Set m2-sourced values
                {
                    for (int r = 0; r < m1.rows; r++)
                    {
                        result[m1.columns + c, r] = m2[c, r];
                    }
                }
                break;
            case ConcMethod.MakeColumn:
                // TODO: Validate matrix dimensions for this operation.
                result = new MatrixCustom(m1.columns, m1.rows + m2.rows);
                break;

            default:
                result = new MatrixCustom();    // Empty matrix
                break;
        }

        return result;
    }
    public static MatrixCustom Concatenate(ConcMethod method, MatrixCustom mSource, params MatrixCustom[] mOthers)
    {
        MatrixCustom result;
        switch (method)
        {
            case ConcMethod.MakeRow:
                // TODO: Validate matrix dimensions for this operation.

                // Evaluate new column-count
                int columns = mSource.columns;
                int rows = mSource.rows;
                for (int i = 0; i < mOthers.Length; i++)
                {
                    columns += mOthers[i].columns;
                }

                result = new MatrixCustom(columns, rows);
                for (int c = 0; c < mSource.columns; c++)
                {
                    for (int r = 0; r < mSource.rows; r++)
                    {
                        result[c, r] = mSource[c, r];
                    }
                }
                int columnSelector = mSource.columns;
                for (int m = 0; m < mOthers.Length; m++)    // Set m1-sourced values
                {
                    for (int c = 0; c < mOthers[m].columns; c++)
                    {
                        for (int r = 0; r < mOthers[m].rows; r++)
                        {
                            result[columnSelector + c, r] = mOthers[m][c, r];
                        }
                    }
                    columnSelector += mOthers[m].columns;
                }
                break;
            case ConcMethod.MakeColumn:
                // TODO: Validate matrix dimensions for this operation.
                throw new NotImplementedException("The row-based matrix concatenation method has not yet been implemented!");
                result = new MatrixCustom(mSource.columns, mSource.rows + mOthers[0].rows);
                break;

            default:
                result = new MatrixCustom();    // Empty matrix
                break;
        }

        return result;
    }

    /// <summary>
    /// Multiplies a column and row vector together which yields a single number.
    /// </summary>
    /// <param name="rowMatrix">A row matrix (must be as long as the other matrix).</param>
    /// <param name="columnMatrix">A column matrix (must be as long as the other matrix).</param>
    /// <returns>The product from a dot-like operation between the two matrices.</returns>
    public static float ColumnRowMult(MatrixCustom rowMatrix, MatrixCustom columnMatrix)
    {
        if (rowMatrix.columns != columnMatrix.rows)
            throw new ArithmeticException($"The row and column matrix do not share opposite dimensions (Row-length: {rowMatrix.columns}, Column-length: {columnMatrix.rows})");

        float result = 0f;

        for (int i = 0; i < rowMatrix.columns; i++)
        {
            result += rowMatrix[i, 0] * columnMatrix[0, i];
        }

        return result;
    }

    /// <summary>
    /// Produces an empty column-vector with zeroes.
    /// </summary>
    /// <param name="length">The length of the column.</param>
    /// <returns>The column-vector.</returns>
    public static MatrixCustom Column(int length)
    {
        return new MatrixCustom(1, length);
    }
    /// <summary>
    /// Produces a column-vector whose column contains the input values.
    /// </summary>
    /// <returns>The column-vector.</returns>
    public static MatrixCustom Column(params float[] values)
    {
        return new MatrixCustom(1, values.Length, values);
    }
    /// <summary>
    /// Produces a column-vector whose column contains the input values.
    /// </summary>
    /// <param name="length">The length of the column.</param>
    /// <param name="value">The value for all the components in the column.</param>
    /// <returns>The column-vector.</returns>
    public static MatrixCustom Column(int length, float value)
    {
        MatrixCustom output = new MatrixCustom(1, length);
        for (int i = 0; i < output.rows; i++)
        {
            output[0,i] = value;
        }
        return output;
    }

    /// <summary>
    /// Produces an empty row-vector with zeroes.
    /// </summary>
    /// <param name="length">The length of the row.</param>
    /// <returns>The row-vector.</returns>
    public static MatrixCustom Row(int length)
    {
        return new MatrixCustom(length, 1);
    }
    /// <summary>
    /// Produces a row-vector whose row contains the input values.
    /// </summary>
    /// <returns>The row-vector.</returns>
    public static MatrixCustom Row(params float[] values)
    {
        return new MatrixCustom(values.Length, 1, values);
    }

    /// <summary>
    /// Produces a square matrix whose diagonal contains the input values.
    /// </summary>
    /// <param name="values">The values on the diagonal from 11 to nn.</param>
    /// <returns>The diagonal matrix.</returns>
    public static MatrixCustom Diagonal(params float[] values)
    {
        // TODO: Implement diagonal matrix constructor
        MatrixCustom result = new MatrixCustom(values.Length, values.Length);
        for (int i = 0; i < values.Length; i++)
        {
            result[i, i] = values[i];
        }
        return result;
    }
    /// <summary>
    /// Produces a square matrix whose diagonal contains the input values.
    /// </summary>
    /// <param name="size">The size (n-by-n) of the matrix.</param>
    /// <param name="value">The value on the diagonal.</param>
    /// <returns>The diagonal matrix</returns>
    public static MatrixCustom Diagonal(int size, float value)
    {
        // TODO: Implement diagonal matrix constructor
        return new MatrixCustom();
    }

    /// <summary>
    /// Converts a Vector3-array to a row-matrix on the form (v3 | v3 | v3 ...).
    /// </summary>
    /// <returns>The stacked row-matrix.</returns>
    public static MatrixCustom Vector3ToRowMatrix(Vector3[] input)
    {
        MatrixCustom result = new MatrixCustom(input.Length, 3);
        for (int v = 0; v < input.Length; v++)
        {
            for (int i = 0; i < 3; i++)
            {
                result[v, i] = input[v][i];
            }
        }

        return result;
    }
    /// <summary>
    /// Converts a Vector3-array to a row-matrix on the form (x,y,z,x,y,z,x...).
    /// </summary>
    /// <returns>The linear row-matrix.</returns>
    public static MatrixCustom Vector3ToLinearMatrix(Vector3[] input)
    {
        MatrixCustom result = new MatrixCustom(input.Length * 3, 1);
        for (int v = 0; v < input.Length; v++)
        {
            for (int i = 0; i < 3; i++)
            {
                result[v * 3 + i, 0] = input[v][i];
            }
        }

        return result;
    }

    public static Vector3[] RowMatrixToVector3(MatrixCustom row)
    {
        if (row.rows % 3 != 0)
            throw new FormatException("The matrix does not have exactly 3 rows. The conversion cannot take place.");

        Vector3[] result = new Vector3[row.columns];
        for (int i = 0; i < row.columns; i++)
        {
            result[i].x = row[i, 0];
            result[i].y = row[i, 1];
            result[i].z = row[i, 2];
        }

        return result;
    }

    /// <summary>
    /// Converts a row-matrix, on the form (x,y,z,x,y,z,x...), to an array of Vector3's on the form (|x y z|, |x y z|...).
    /// </summary>
    /// <param name="linearRow">The row-matrix (if you have a column-matrix, just transpose it).</param>
    /// <returns>The Vector3-array.</returns>
    public static Vector3[] LinearMatrixToVector3(MatrixCustom linearRow)
    {
        if (linearRow.columns % 3 != 0)
            throw new FormatException("The length of the row-matrix is not a multiple of 3. The conversion cannot take place.");

        Vector3[] result = new Vector3[linearRow.columns / 3];
        for (int i = 0; i < linearRow.columns / 3; i++)
        {
            result[i].x = linearRow[i * 3 + 0, 0];
            result[i].y = linearRow[i * 3 + 1, 0];
            result[i].z = linearRow[i * 3 + 2, 0];
        }

        return result;
    }

    public bool HasNaNs()
    {
        for (int c = 0; c < this.columns; c++)
        {
            for (int r = 0; r < this.rows; r++)
            {
                if (float.IsNaN(this[c, r]))
                    return true;
            }
        }
        return false;
    }

    public override string ToString()
    {
        string output = "\n";

        for (int r = 0; r < rows; r++)
        {
            output += "[";
            for (int c = 0; c < columns; c++)
            {
                output += this[c, r].ToString(CultureInfo.InvariantCulture);
                if (c < columns - 1)
                    output += "  ";
            }
            output += "]";

            if (r < rows - 1)
                output += '\n';
        }

        return output;
    }
}
