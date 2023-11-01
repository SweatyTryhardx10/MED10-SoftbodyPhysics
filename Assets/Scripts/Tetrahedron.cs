using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
    /// <summary>ID for this node in the global system.</summary>
    public int globalID;

    public Vector3 initPos;
    public Vector3 pos;
    public Vector3 VirtualDisplace { get => pos - initPos; }

    public float[] warpedStiffnessRow;  // A portion of the global system's *warped* stiffness matrix
    public Vector3 offsetForceVector;   // What is this??? (The position at which the force as applied after stiffness warping/displacement?)
    public float mass;

    public Node(int globalID, Vector3 initPosition)
    {
        this.globalID = globalID;
        this.initPos = initPosition;
        this.pos = initPos;
    }
}

public class Tetrahedron
{
    public Node[] nodes { get; private set; }
    private float[] barycentricWeights;

    /// <summary>The deformation for all 4 nodes in the tetrahedron.</summary>
    public Vector3[] deformation { get; }
    public MatrixCustom stiffnessMatrix { get; private set; }   // Check literature for size and computation
    public MatrixCustom lumpedMassMatrix { get; private set; }  // Check literature for size and computation

    public Vector3[] forceVector { get; private set; }

    public Matrix4x4 orientation { get; private set; }          // Check literature for size and computation

    public float massDensity;
    public float Volume => ComputeVolume();  // TODO: Implement volume calculation

    public Tetrahedron(Node n1, Node n2, Node n3, Node n4)
    {
        nodes = new Node[4];
        nodes[0] = n1;
        nodes[1] = n2;
        nodes[2] = n3;
        nodes[3] = n4;

        orientation = Matrix4x4.identity;   // Initialize orientation as identity matrix

        ComputeBarycentricWeights();
        ComputeWarpedStiffness();
        ComputeLumpedMass();

        forceVector = new Vector3[nodes.Length];
        for (int i = 0; i < forceVector.Length; i++)
        {
            forceVector[i] = Vector3.down * 9.82f;
        }
    }

    private void ComputeBarycentricWeights()
    {
        // TODO: Compute weights based off of current vertex information
        barycentricWeights = new float[4];
    }

    public void ComputeStiffnessMatrix()
    {
        // TODO: Compute stiffness matrix
        // Element stiffness matrix (Ke):
        //  S = A strain operator (for defining stretching and shearing -> 11, 22, 33 is stretch, and 14, 24, 15, 35, 26, 36 is shearing)
        //  N = a 3x3 diagonal matrix containing barycentric coordinates on the diagonal
        //  B = S*N
        //  D = Isotropic elasticity matrix (describes material properties: instretchability (Y), and contraction behaviour (v))
        //  Ve = volume
        //  Ke = B.transpose * D * B * Ve

        MatrixCustom S = StrainOperator();

        float n0 = GetNaturalCoordsFromWorldPoint(nodes[0].pos).x;
        float n1 = GetNaturalCoordsFromWorldPoint(nodes[1].pos).y;
        float n2 = GetNaturalCoordsFromWorldPoint(nodes[2].pos).z;
        float n3 = GetNaturalCoordsFromWorldPoint(nodes[3].pos).w;
        MatrixCustom N0 = MatrixCustom.Diagonal(n0, n0, n0);
        MatrixCustom N1 = MatrixCustom.Diagonal(n1, n1, n1);
        MatrixCustom N2 = MatrixCustom.Diagonal(n2, n2, n2);
        MatrixCustom N3 = MatrixCustom.Diagonal(n3, n3, n3);
        MatrixCustom N = MatrixCustom.Concatenate(MatrixCustom.ConcMethod.MakeRow, N0, N1, N2, N3);

        MatrixCustom B = S * N;

        // TODO: Implement the improved B-evaluation technique later!
        // MatrixCustom B0 = new MatrixCustom(3, 9);   // TODO: Insert relevant values in here!!!
        // MatrixCustom B1 = new MatrixCustom(3, 9);   // TODO: Insert relevant values in here!!!
        // MatrixCustom B2 = new MatrixCustom(3, 9);   // TODO: Insert relevant values in here!!!
        // MatrixCustom B3 = new MatrixCustom(3, 9);   // TODO: Insert relevant values in here!!!
        // MatrixCustom B = MatrixCustom.Concatenate(MatrixCustom.ConcMethod.Columns, B0, B1, B2, B3);


        MatrixCustom D = IsotropicElasticityMatrix();

        // if (Time.frameCount % 100 == 0)
        // {
        //     Debug.Log("S: " + S);
        //     Debug.Log("N: " + N);
        //     Debug.Log("B: " + B);

        //     Debug.Log("D: " + D);
        // }

        stiffnessMatrix = B.transpose * D * B * Volume;
    }

    /// <summary>A strain operator (for defining stretching and shearing -> 11, 22, 33 is stretch, and 14, 24, 15, 35, 26, 36 is shearing)<br />CAUTION: Currently incorrect!</summary>
    private MatrixCustom StrainOperator()
    {
        Vector3[] u = deformation;
        float eps11 = 1f;   // What is the value of this?
        float eps22 = 1f;   // What is the value of this?
        float eps33 = 1f;   // What is the value of this?
        float gamm12 = 1f;  // What is the value of this?
        float gamm13 = 1f;  // What is the value of this?
        float gamm23 = 1f;  // What is the value of this?

        return new MatrixCustom(3, 6,
                eps11, 0, 0,
                0, eps22, 0,
                0, 0, eps33,
                gamm13, gamm12, 0,
                gamm23, 0, gamm12,
                0, gamm23, gamm13
            ); // This is currently, likely, incorrect!
    }

    private MatrixCustom IsotropicElasticityMatrix()
    {
        // D = Isotropic elasticity matrix (describes material properties: instretchability (Y), and contraction behaviour (v))
        float Y = 0.05f;    // Rubber coefficient (0.05 GPa)?
        float v = 0.5f;     // Between [-1; 0.5]

        float scalar = Y / ((1 + v) * (1 - 2 * v));

        float D0 = 1 - v;
        float D1 = v;
        float D2 = (1f - 2f * v) / 2f;

        MatrixCustom matrix = new MatrixCustom(
            new float[] { D0, D1, D1, 0,  0,  0 },
            new float[] { D1, D0, D1, 0,  0,  0 },
            new float[] { D1, D1, D0, 0,  0,  0 },
            new float[] { 0,  0,  0,  D2, 0,  0 },
            new float[] { 0,  0,  0,  0, D2,  0 },
            new float[] { 0,  0,  0,  0,  0, D2 }
        );

        return Mathf.Min(10f, scalar) * matrix;     // NOTE: The scalar is infinity when 'v' is at it's max potential value (which is 0.5). Should this be the case?
    }

    private void ComputeWarpedStiffness()
    {
        // TODO: Compute warped stiffness matrix
        for (int i = 0; i < nodes.Length; i++)
        {
            for (int j = 0; j < nodes.Length; j++)
            {
                if (i >= j)
                {
                    // var tmp = orientation * stiffnessMatrix * orientation.inverse;  // TODO: Implement operator(s) between MatrixCustom and other matrix classes

                    if (j > i)
                    {

                    }
                }
            }
        }
    }

    private void ComputeLumpedMass()
    {
        // Compute lumped mass
        //  p = mass density
        //  Ve = volume
        //  Mn = p * (Ve/4)
        foreach (Node n in nodes)
        {
            n.mass = massDensity * (Volume / 4);
        }
    }

    private float ComputeVolume()
    {
        return 1f;  // DEBUG DEBUG DEBUG

        // "Volume" Matrix
        Vector3 column0 = nodes[1].pos - nodes[0].pos;
        Vector3 column1 = nodes[2].pos - nodes[0].pos;
        Vector3 column2 = nodes[3].pos - nodes[0].pos;

        // Determinant computation
        float det0 = (column1.y * column2.z - column2.y * column1.z);
        float det1 = (column0.y * column2.z - column2.y * column0.z);
        float det2 = (column0.y * column1.z - column1.y * column0.z);
        float det = column0.x * det0 - column1.x * det1 - column2.x * det2;

        return det;
    }

    public void ComputeOrientation()
    {

    }

    public void RecalculateStiffnessWarping()
    {
        // TODO: Recalculate stiffness warping (...after position updates, according to the literature)
    }

    // TODO!!!: Compute natural coordinates from world-space point
    // WARNING: (System is currently broken because the values become NaN!!!)
    public Vector4 GetNaturalCoordsFromWorldPoint(Vector3 point)
    {
        // Points
        Vector4 p = new Vector4(point.x, point.y, point.z, 1f);
        Vector4 a = new Vector4(nodes[0].pos.x, nodes[0].pos.y, nodes[0].pos.z, 1f);
        Vector4 b = new Vector4(nodes[1].pos.x, nodes[1].pos.y, nodes[1].pos.z, 1f);
        Vector4 c = new Vector4(nodes[2].pos.x, nodes[2].pos.y, nodes[2].pos.z, 1f);
        Vector4 d = new Vector4(nodes[3].pos.x, nodes[3].pos.y, nodes[3].pos.z, 1f);

        // Determinants
        float dBase = new Matrix4x4(a, b, c, d).determinant;
        float d0 = new Matrix4x4(p, b, c, d).determinant;
        float d1 = new Matrix4x4(a, p, c, d).determinant;
        float d2 = new Matrix4x4(a, b, p, d).determinant;
        float d3 = new Matrix4x4(a, b, c, p).determinant;

        // Weights
        float u = d0 / dBase;
        float v = d1 / dBase;
        float w = d2 / dBase;
        float x = d3 / dBase;

        float sum = u + v + w + x;

        // Debug.Log($"({u}, {v}, {w}, {x}) = {sum}");

        // return new Vector4(u, v, w, x);
        return new Vector4(0.5f, 0.5f, 0.5f, 0.5f);
    }
}
