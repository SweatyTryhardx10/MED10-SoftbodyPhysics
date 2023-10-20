using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class SoftbodyFEM : MonoBehaviour
{
    [System.Flags]
    enum DebugFlag
    {
        Elements = 1 << 1,
        MeshVertices = 1 << 2,
        VolumeVertices = 1 << 3,
    }

    [Header("Settings")]
    [Tooltip("The total mass of the softbody distributed evenly amongst all elements.")]
    [SerializeField] private float mass;
    [SerializeField] private Mesh femMesh;

    [Header("Debugging")]
    [SerializeField] private DebugFlag debugFlags;

    private Tetrahedron[] elements;
    private Node[] globalNodes;
    // Key is global vertex, and value is an array of integer values pointing to entries in the elements-array.

    private void Awake()
    {
        // Initialize current mesh for softbody mechanics (FEM) by baking elements for the entire mesh volume
        elements = MeshUtil.MeshToTetrahedrons(femMesh, transform.position, out globalNodes);
    }

    private void Start()
    {
        // DEBUG DEBUG DEBUG
        // MatrixCustom m1 = new MatrixCustom(4, 4,
        //     4f, 2f, 1f, -6f,
        //     1f, 8f, 1f, 16f,
        //     2f, 78f, 2f, 8f,
        //     45f, 100f, 6f, -1f
        // );
        // MatrixCustom m2 = new MatrixCustom(2, 4,
        //     11f, -12f,
        //     2f, 3.5f,
        //     1f, 5f,
        //     120f, 17f
        // );

        // Debug.Log(m1);
        // Debug.Log(m1.transpose);
        // Debug.Log(m2);
        // Debug.Log(m2.transpose);
        // Debug.Log(m1 * m2);
    }

    private void FixedUpdate()
    {
        // TODO: Solve Finite Element Method (FEM)-based softbody mechanics
        SolveElements();

        // TODO: Update the mesh coupled to the system
    }

    private void SolveElements()
    {
        // Compute stiffness matrices for all elements
        for (int i = 0; i < elements.Length; i++)
        {
            elements[i].ComputeStiffnessMatrix();
        }

        // TODO: Assemble global stiffness matrix
        MatrixCustom K = new MatrixCustom(globalNodes.Length, globalNodes.Length);
        // TODO: 'Compute' the force vector for the global system
        Vector3[] globalForceVector = new Vector3[globalNodes.Length];
        foreach (Tetrahedron t in elements)
        {
            for (int m = 0; m < t.nodes.Length; m++)
            {
                for (int n = 0; n < t.nodes.Length; n++)
                {
                    K[t.nodes[n].globalID, t.nodes[m].globalID] += t.stiffnessMatrix[n, m];
                }

                globalForceVector[t.nodes[m].globalID] += t.forceVector[m];
            }
        }

        // Mass matrix (lumped) (ALSO - IGNORE FOR NOW)
        MatrixCustom M = MatrixCustom.Diagonal(globalNodes.Length, 1f);

        // Damping matrix (IGNORE FOR NOW)
        MatrixCustom C = new MatrixCustom();

        // TODO: Solve for next-frame velocity (using conjugate gradient solver - damn)
        Vector3[] vNext = new Vector3[0];

        // Global displacement vector
        Vector3[] u = MatrixCustom.MatrixToVector3(
                        MatrixCustom.CGDSolver(K, MatrixCustom.Vector3ToMatrix(globalForceVector), MatrixCustom.Column(globalNodes.Length * 3)
                    ));

        // TODO: Update the positions for all global nodes
        for (int i = 0; i < elements.Length; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                int nodeID = elements[i].nodes[j].globalID;

                // Add displacement to node position
                elements[i].nodes[j].pos += u[nodeID];
            }
        }
    }

    private void OnDrawGizmos()
    {
        if (debugFlags.HasFlag(DebugFlag.Elements))
        {
            if (elements != null)
            {
                int tetHighlight = Mathf.FloorToInt((Time.time * 0.5f) % elements.Length);

                // MeshUtil.TetrahedronHandle(elements[tetHighlight], Color.HSVToRGB((float)tetHighlight / elements.Length, 1f, 1f).WithAlpha(0.3f));

                for (int i = 0; i < elements.Length; i++)
                {
                    if (i == tetHighlight)
                        continue;
                    if (i % 10 != 0)
                        continue;

                    MeshUtil.TetrahedronHandle(elements[i], Color.HSVToRGB((i / 512f) % 1f, 1f, 1f).WithAlpha(0.3f));
                }
            }
        }
        if (debugFlags.HasFlag(DebugFlag.MeshVertices))
        {

        }
        if (debugFlags.HasFlag(DebugFlag.VolumeVertices))
        {

        }
    }
}
