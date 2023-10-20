using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
    /// <summary>ID for this node in the global system.</summary>
    public int globalID;
    
    public Vector3 pos;

    public float[] warpedStiffnessRow;  // A portion of the global system's *warped* stiffness matrix
    public Vector3 offsetForceVector;   // What is this??? (The position at which the force as applied after stiffness warping/displacement?)
    public float mass;

    public Node(int globalID, Vector3 initPosition)
    {
        this.globalID = globalID;
        pos = initPosition;
    }
}

public class Tetrahedron
{
    public Node[] nodes { get; private set; }
    private float[] barycentricWeights;

    public Vector3[] DeformedNodes { get; }
    public MatrixCustom stiffnessMatrix { get; private set; }   // Check literature for size and computation
    public MatrixCustom lumpedMassMatrix { get; private set; }  // Check literature for size and computation

    public Vector3[] forceVector { get; private set; }

    public Matrix4x4 orientation { get; private set; }          // Check literature for size and computation

    public float massDensity;
    public float Volume => 0f;  // TODO: Implement volume calculation

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
        MatrixCustom N = MatrixCustom.Diagonal(
            1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f  // THIS IS CORRECT?!
        );
        MatrixCustom B = S * N;

        MatrixCustom D = IsotropicElasticityMatrix();

        stiffnessMatrix = B.transpose * D * B * Volume;
    }

    /// <summary>A strain operator (for defining stretching and shearing -> 11, 22, 33 is stretch, and 14, 24, 15, 35, 26, 36 is shearing)<br />CAUTION: Currently incorrect!</summary>
    private MatrixCustom StrainOperator()
    {
        float eps11 = 0f;
        float eps22 = 0f;
        float eps33 = 0f;
        float gamm12 = 0f;
        float gamm13 = 0f;
        float gamm23 = 0f;

        return new MatrixCustom(3, 6, new float[]{
                eps11, 0, 0,
                0, eps22, 0,
                0, 0, eps33,
                gamm13, gamm12, 0,
                gamm23, 0, gamm12,
                0, gamm23, gamm13
            }); // This is currently, likely, incorrect!
    }

    private MatrixCustom IsotropicElasticityMatrix()
    {
        // D = Isotropic elasticity matrix (describes material properties: instretchability (Y), and contraction behaviour (v))
        float Y = 0.05f;    // Rubber coefficient (0.05 GPa)?
        float v = 0.5f;     // Between [-1; 0.5]

        float scalar = Y / ((1 + v) * (1 - 2 * v));

        MatrixCustom matrix = new MatrixCustom(
            new float[] { 1 - v, v, v, 0, 0, 0 },
            new float[] { v, 1 - v, v, 0, 0, 0 },
            new float[] { v, v, 1 - v, 0, 0, 0 },
            new float[] { 0, 0, 0, (1f - 2f * v) / 2f, 0, 0 },
            new float[] { 0, 0, 0, 0, (1f - 2f * v) / 2f, 0 },
            new float[] { 0, 0, 0, 0, 0, (1f - 2f * v) / 2f }
        );

        return scalar * matrix;
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

    public void ComputeOrientation()
    {

    }

    public void RecalculateStiffnessWarping()
    {
        // TODO: Recalculate stiffness warping (...after position updates, according to the literature)
    }
}
