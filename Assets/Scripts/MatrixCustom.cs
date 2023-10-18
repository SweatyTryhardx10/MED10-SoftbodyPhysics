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
                throw new IndexOutOfRangeException($"index {x} is larger than {this}'s column count");
            if (y >= rows)
                throw new IndexOutOfRangeException($"index {y} is larger than {this}'s row count");

            return matrix[x, y];
        }
        set
        {
            if (x >= columns)
                throw new IndexOutOfRangeException($"index {x} is larger than {this}'s column count");
            if (y >= rows)
                throw new IndexOutOfRangeException($"index {y} is larger than {this}'s row count");

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
    /// Computes the inverse of this matrix via a Conjugate Gradient Descent solver. (Right???)
    /// </summary>
    /// <returns>The resulting inverse of this matrix.</returns>
    public MatrixCustom GetInverseCGD()
    {
        return new MatrixCustom();
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

    /// <summary>
    /// Produces a square matrix whose diagonal contains the input values.
    /// </summary>
    /// <param name="values">The values on the diagonal from 11 to nn.</param>
    /// <returns></returns>
    public static MatrixCustom Diagonal(params float[] values)
    {
        // TODO: Implement diagonal matrix constructor
        return new MatrixCustom();
    }

    public override string ToString()
    {
        string output = "\nMatrix:\n";

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
