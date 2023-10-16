using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public static class MeshUtil
{
    [System.Serializable]
    public struct MeshData
    {
        public Vector3[] vertices;
        public int[] triangles;
        public Vector3[] normals;

        /// <summary>
        ///     Stores unordered sets of neighbours for each vertex.<br />
        ///     NOTE: Selectors: [vertex index] [distance] [neigbour index]
        /// </summary>
        public int[,][] vertexNeighbourSets { get; private set; }

        /// <summary>Computes and stores neighbourhood relationships between vertices in 'vertexNeighbourSets'.</summary>
        public void BakeNeighbours()
        {
            // Initialize neighbour array
            vertexNeighbourSets = new int[vertices.Length, 1][];

            // For each triangle vertex, check connected vertices and 'mark' them as neighbours if not already a neighbour
            List<int> vertexIndexHistory = new List<int>();
            for (int i = 0; i < triangles.Length; i++)
            {
                // Don't check if vertex has already been evaluated
                if (!vertexIndexHistory.Contains(triangles[i]))
                    vertexIndexHistory.Add(triangles[i]);
                else
                    continue;

                List<int> neighbourIndeces = new List<int>();
                for (int j = 0; j < triangles.Length / 3; j++)
                {
                    // Check whether the triangle contains the ith-vertex
                    int sourceVertex = triangles[i];
                    int[] triangle = new int[] {
                        triangles[j * 3],
                        triangles[j * 3 + 1],
                        triangles[j * 3 + 2]
                        };
                    bool containsSourceVertex = (triangle[0] == sourceVertex) || (triangle[1] == sourceVertex) || (triangle[2] == sourceVertex);
                    // Debug.Log("Contains source vertex: " + containsSourceVertex);
                    if (containsSourceVertex)
                    {
                        // Determine valid neighbours from triangle
                        for (int k = 0; k < triangle.Length; k++)
                        {
                            bool isSource = triangle[k] == sourceVertex;
                            bool isNewNeighbour = !neighbourIndeces.Contains(triangle[k]);
                            if (!isSource && isNewNeighbour)
                            {
                                // Add vertex as neighbour to source
                                neighbourIndeces.Add(triangle[k]);
                            }
                        }
                    }
                }

                // Add neighbour-set to public array
                vertexNeighbourSets[triangles[i], 0] = neighbourIndeces.ToArray();
            }

            Debug.Log($"Vertex 0 neighbour count: {vertexNeighbourSets[0, 0].Length}");
        }

        public int[] GetNeighboursAtDistance(int sourceVertexIndex, int distance)
        {
            List<int> neighbourOutput = new List<int>();

            List<int> neighbourQueue = new List<int>() { sourceVertexIndex };
            Stack<int> neighbourFrontier = new Stack<int>();
            List<int> traversedVertices = new List<int>();

            int depth = 1;
            while (neighbourQueue.Count > 0)
            {
                // Stop loop if target distance has been reached
                if (depth > distance)
                    break;

                // Fill up frontier (...then clear the queue)
                neighbourQueue.ForEach(n => neighbourFrontier.Push(n));
                neighbourQueue.Clear();

                Debug.Log($"Depth: {depth} (Frontier count: {neighbourFrontier.Count})");

                // Accumulate valid neighbours from the current frontier
                while (neighbourFrontier.TryPop(out int neighbourIndex))
                {
                    // Only check non-traversed neighbours
                    if (!traversedVertices.Contains(neighbourIndex))
                    {
                        traversedVertices.Add(neighbourIndex);

                        int[] neighbours = vertexNeighbourSets[neighbourIndex, 0];
                        for (int i = 0; i < neighbours.Length; i++)
                        {
                            if (!neighbourFrontier.Contains(neighbours[i]) && !traversedVertices.Contains(neighbours[i]))
                            {
                                if (depth == distance)
                                    neighbourOutput.Add(neighbours[i]);     // Add to output
                                else
                                    neighbourQueue.Add(neighbours[i]);
                            }
                        }
                    }
                }

                Debug.Log($"Depth: {depth} - Queue count: {neighbourQueue.Count}");

                // Iterate depth
                depth++;
            }

            return neighbourOutput.ToArray();
        }
    }

    /// <summary>
    /// Create a face.
    /// </summary>
    /// <param name="normal">The normal in which the face should look.</param>
    /// <param name="subdivisions">The subdivisions the face should contain.</param>
    /// <returns>Vertex and triangle data that describes the face created.<br/>Neighbour-relationship is NOT baked by default; use BakeNeighbours().</returns>
    public static MeshData CreateFace(Vector3 normal, int subdivisions)
    {
        int width = subdivisions + 2;
        int height = subdivisions + 2;
        Vector3[] v = new Vector3[width * height];
        int[] t = new int[6 * (width - 1) * (height - 1)];

        int idxT = 0;

        // Face transformation (based on normal)
        Quaternion rotation = Quaternion.LookRotation(normal, Vector3.up);
        Vector3 scale = Vector3.one / (subdivisions + 1);
        Matrix4x4 basisMatrix = Matrix4x4.TRS(Vector3.zero, rotation, scale);

        // Vertices (for face)
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int idxV = y * width + x;
                v[idxV] = basisMatrix.MultiplyPoint3x4(new Vector3(x, y, 0) - new Vector3(width - 1, height - 1, 0) / 2f);

                // Triangles
                if (x < width - 1 && y < height - 1)
                {
                    // Triangle 1
                    t[idxT + 0] = idxV;
                    t[idxT + 1] = idxV + width + 1;
                    t[idxT + 2] = idxV + width;

                    // Triangle 2
                    t[idxT + 3] = idxV;
                    t[idxT + 4] = idxV + 1;
                    t[idxT + 5] = idxV + width + 1;

                    idxT += 6;
                }
            }
        }

        // Return face
        return new MeshData() { vertices = v, triangles = t };
    }

    /// <summary>
    /// Create a cube.
    /// </summary>
    /// <param name="subdivisions">The subdivisions for each face of the cube.</param>
    /// <returns>Vertex and triangle data that describes the cube created.<br/>Neighbour-relationship is NOT baked by default; use BakeNeighbours().</returns>
    public static MeshData CreateCube(int subdivisions)
    {
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();
        Vector3[] faceDirection = new Vector3[] { Vector3.up, Vector3.down, Vector3.left, Vector3.right, Vector3.forward, Vector3.back };

        // Generate sides
        for (int i = 0; i < faceDirection.Length; i++)
        {
            var face0 = CreateFace(faceDirection[i], subdivisions);
            triangles.AddRange(face0.triangles.ValueOffset(vertices.Count));
            vertices.AddRange(face0.vertices.ValueOffset(faceDirection[i] / 2f));
        }

        return new MeshData() { vertices = vertices.ToArray(), triangles = triangles.ToArray() };
    }

    /// <summary>
    /// Create a sphere (modified cube).
    /// </summary>
    /// <param name="subdivisions">The subdivisions for the sphere.</param>
    /// <returns>Vertex and triangle data that describes the sphere created.<br/>Neighbour-relationship is NOT baked by default; use BakeNeighbours().</returns>
    public static MeshData CreateSphere(int subdivisions)
    {
        var cube = CreateCube(subdivisions);
        Vector3[] vertices = cube.vertices.Select(v => v.normalized).ToArray();

        return new MeshData() { vertices = vertices, triangles = cube.triangles };
    }

    public static Tetrahedron[] MeshToTetrahedrons(Mesh mesh, out Node[] globalNodes)
    {
        return MeshToTetrahedrons(mesh, Vector3.zero, out globalNodes);
    }

    public static Tetrahedron[] MeshToTetrahedrons(Mesh mesh, Vector3 offset, out Node[] globalNodes)
    {
        // Retrieve mesh data (vertices and triangles)
        Vector3[] vertices = mesh.vertices;
        int[] quadIndices = mesh.GetTriangles(0);
        
        // Create global node array
        globalNodes = new Node[vertices.Length];
        for (int i = 0; i < globalNodes.Length; i++)    // One node per vertex
        {
            globalNodes[i] = new Node(i, offset + vertices[i]);
        }

        // Make tetrahedrons from mesh
        Tetrahedron[] tets = new Tetrahedron[quadIndices.Length / 6];
        for (int i = 0; i < quadIndices.Length / 6; i++)
        {
            // Find the 4 unique vertex indices among the 6 indices from the triangles
            int[] vertUIDs = new int[4] { -1, -1, -1, -1 };   // Unique vertex indices
            int indicesFound = 0;
            for (int j = 0; j < 6; j++)
            {
                int index = quadIndices[i * 6 + j];
                if (!vertUIDs.Contains(index))
                {
                    vertUIDs[indicesFound] = index;
                    indicesFound++;
                }
            }

            // 'Add' tetrahedron to list
            // TODO: Initialize a global node array. These nodes should then be used in the creation of new tetrahedrons!
            tets[i] = new Tetrahedron(
                globalNodes[vertUIDs[0]],
                globalNodes[vertUIDs[1]],
                globalNodes[vertUIDs[2]],
                globalNodes[vertUIDs[3]]
            );
        }

        return tets;
    }


    // ===========================
    // ==== EXTENSION METHODS ====
    // ===========================
    public static Vector3[] ValueOffset(this Vector3[] array, Vector3 offset)
    {
        Vector3[] offsetArray = new Vector3[array.Length];
        for (int i = 0; i < array.Length; i++)
        {
            offsetArray[i] = array[i] + offset;
        }

        return offsetArray;
    }

    public static int[] ValueOffset(this int[] array, int offset)
    {
        int[] offsetArray = new int[array.Length];
        for (int i = 0; i < array.Length; i++)
        {
            offsetArray[i] = array[i] + offset;
        }

        return offsetArray;
    }

    public static Color WithAlpha(this Color color, float alpha)
    {
        return new Color(color.r, color.g, color.b, alpha);
    }

    public static void TetrahedronHandle(Tetrahedron tet, Color color)
    {
        UnityEditor.Handles.color = color;
        for (int i = 0; i < 4; i++)
        {
            int i0 = ((byte)(i + 1)).GetBit(3) ? 1 : 0;
            int i1 = 1 + (i + 0) % 3;
            int i2 = 1 + (i + 1) % 3;

            Vector3[] triangle = new Vector3[] {
                tet.nodes[i0].pos,
                tet.nodes[i1].pos,
                tet.nodes[i2].pos
            };

            UnityEditor.Handles.DrawAAConvexPolygon(triangle);
        }
    }
}