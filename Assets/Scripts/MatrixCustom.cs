using System;
using System.Collections;
using System.Collections.Generic;
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
        this.rows = rows;
        this.columns = columns;

        matrix = new float[rows, columns];
        for (int i = 0; i < columns; i++)
        {
            for (int j = 0; j < rows; j++)
            {
                matrix[i, j] = values[i * columns + j];
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
    /// <param name="x">The row index in the matrix.</param>
    /// <param name="y">The column index in the matrix.</param>
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

    public MatrixCustom transpose
    {
        get
        {
            // TODO: Implement transpose short-hand
            return new MatrixCustom();
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

    public static MatrixCustom operator *(MatrixCustom m1, MatrixCustom m2)
    {
        // TODO: Implement matrix multiplication for arbitrary matrix sizes
        return new MatrixCustom();
    }

    /// <summary>
    /// Computes the inverse of this matrix via a Conjugate Gradient Descent solver.
    /// </summary>
    /// <returns>The resulting inverse of this matrix.</returns>
    public MatrixCustom GetInverseCGD()
    {
        return new MatrixCustom();
    }

    public static MatrixCustom operator *(MatrixCustom m1, float scalar)
    {
        // TODO: Implement scalar multiplication
        return m1;
    }
    public static MatrixCustom operator *(float scalar, MatrixCustom m1)
    {
        // TODO: Implement scalar multiplication
        return m1;
    }
    public static Vector3[] operator *(MatrixCustom m1, Vector3[] vector)
    {
        // TODO: Implement matrix multiplication for lists of 3D-vectors on the form [x,y,z,x,y,z,x...]

        // TODO: ...and then convert it back to the form [|xyz|, |xyz|, ...]

        return vector;
    }
    public static MatrixCustom operator /(MatrixCustom m1, float divider)
    {
        // TODO: Implement scalar division
        return m1;
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
}
