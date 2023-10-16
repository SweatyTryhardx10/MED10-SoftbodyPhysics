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

    private void FixedUpdate()
    {
        // TODO: Solve Finite Element Method (FEM)-based softbody mechanics
        SolveElements();
        
        // TODO: Update the mesh coupled to the system
    }

    private void SolveElements()
    {
        
        // TODO: Assemble global stiffness matrix
        MatrixCustom K = new MatrixCustom();
        
        // Damping matrix (IGNORE FOR NOW)
        MatrixCustom C = new MatrixCustom();
        
        // Mass matrix (lumped) (ALSO - IGNORE FOR NOW)
        MatrixCustom M = new MatrixCustom();
        
        
        // TODO: 'Compute' the force vector for the global system
        Vector3[] globalForceVector = new Vector3[globalNodes.Length];
        
        // TODO: Solve for next-frame velocity (using conjugate gradient solver - damn)
        Vector3[] vNext = new Vector3[0];
        
        // Global displacement vector
        Vector3[] u = K.inverse * globalForceVector;
        
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
