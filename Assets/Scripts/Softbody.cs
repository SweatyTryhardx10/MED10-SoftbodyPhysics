using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using UnityEditor;
using System;

public class Softbody : MonoBehaviour
{
    private class Particle
    {
        public const float SKINWIDTH = 0.01f;

        public Vector3 position;
        public Vector3 velocity;
        public float mass;

        public void Move(Vector3 amountToMove)
        {
            position += amountToMove;
        }
    }

    private class Spring
    {
        // Hooke's law for damped spring connected to two particles, a and b:
        // F_a = -F_b = -[K_s * (|L| - R) + K_d * (i * L / |L|)] * (L / |L|)
        // K_s: Spring Constant
        // L: Vector from A to B particle
        // i = v_a - v_b: Vector of instantaneous change
        // R: Rest length
        public float springConstant;
        public float dampingConstant;
        public float restLength;

        public readonly Particle particle1;
        public readonly Particle particle2;

        public Spring(Particle p1, Particle p2)
        {
            particle1 = p1;
            particle2 = p2;
        }

        public void Tick()
        {
            // Apply force to particle 1
            Vector3 L = particle1.position - particle2.position;
            Vector3 i = particle1.velocity - particle2.velocity;
            Vector3 acceleration = -(springConstant * (L.magnitude - restLength) + dampingConstant * (Vector3.Dot(i, L) / L.magnitude)) * L.normalized;
            particle1.velocity += acceleration * Time.fixedDeltaTime;

            // Apply force to particle 2
            particle2.velocity -= acceleration * Time.fixedDeltaTime;
        }
    }

    enum MeshPrimitive
    {
        Plane,
        Cube,
        Sphere,
        Reference
    }

    [Header("Settings")]
    [SerializeField] private MeshPrimitive primitive;
    public int meshDensity = 0;
    public float meshSize = 1f;
    public float mass = 1f;
    [SerializeField] private float gravity = 9.82f;
    public Material material;
    public Mesh meshRef;

    [Header("Spring System Settings")]
    public float structuralSpringStrength = 1f;
    public float structuralSpringDamping = 0.1f;
    public float bendingSpringStrength = 1f;
    public float bendingSpringDamping = 0.1f;

    private Mesh softMesh;
    private MeshUtil.MeshData meshData;
    private MeshRenderer meshRenderer;
    private MeshFilter meshFilter;

    private Dictionary<int, Particle> particles = new Dictionary<int, Particle>();
    private List<Spring> springs = new List<Spring>();

    [Flags]
    enum DebugFlag
    {
        Particles = 1 << 1,
        Triangles = 1 << 2,
        Vertices = 1 << 3,
        Springs = 1 << 4,
    }

    [Header("Debugging")]
    [SerializeField] private bool showDebug;
    [SerializeField] private DebugFlag debugFlags;
    [SerializeField] private int debugVisualizeVertexIndexInfo;

    private void Awake()
    {
        InitializeComponents();

        // Assign material to renderer
        meshRenderer.sharedMaterial = material;

        // Generate mesh used for softbody physics
        softMesh = GenerateMesh();
        Debug.Log($"Soft Mesh Vertex Count: {softMesh.vertexCount}");

        // Assign mesh to filter
        meshFilter.mesh = softMesh;

        GenerateParticles();
        GenerateSprings();
    }

    private void OnValidate()
    {
        if (Application.isPlaying)
            return;

        InitializeComponents();

        RegenerateMesh();
    }

    private void InitializeComponents()
    {
        // Initialize components for rendering
        if (!meshRenderer)
        {
            if (!TryGetComponent<MeshRenderer>(out meshRenderer))
                meshRenderer = gameObject.AddComponent<MeshRenderer>();

        }
        if (!meshFilter)
        {
            if (!TryGetComponent<MeshFilter>(out meshFilter))
                meshFilter = gameObject.AddComponent<MeshFilter>();
        }
    }

    private void FixedUpdate()
    {
        // Softbody physics (w. mass-spring system)
        EulerIntegrationPhysics();
        
        // Move transform alogn with particle system
        Vector3 particleSystemPos = Vector3.zero;
        foreach (Particle p in particles.Values) { particleSystemPos += p.position; }
        particleSystemPos /= particles.Count;
        if (!float.IsNaN(particleSystemPos.x))
            transform.position = particleSystemPos;
    }

    [ContextMenu("Regenerate Soft Mesh")]
    public void RegenerateMesh()
    {
        // Generate mesh used for softbody physics
        softMesh = GenerateMesh();

        // Assign mesh to filter
        if (meshFilter)
            meshFilter.mesh = softMesh;
    }

    private Mesh GenerateMesh()
    {
        Mesh mesh = new Mesh();

        // Create new mesh
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();
        List<Vector3> normals = new List<Vector3>();
        switch (primitive)
        {
            case MeshPrimitive.Plane:
                meshData = MeshUtil.CreateFace(Vector3.up, meshDensity);
                break;
            case MeshPrimitive.Cube:
                meshData = MeshUtil.CreateCube(meshDensity);
                break;
            case MeshPrimitive.Sphere:
                meshData = MeshUtil.CreateSphere(meshDensity);
                break;
            case MeshPrimitive.Reference:
                meshData = new MeshUtil.MeshData() { vertices = meshRef.vertices, triangles = meshRef.triangles, normals = meshRef.normals };
                break;
            default:
                Debug.LogError("Invalid MeshPrimitive enum was set. Please verify integrity of variable.");
                break;
        }
        meshData.BakeNeighbours();  // Bake neighbour set(s)
        vertices.AddRange(meshData.vertices.Select(v => v * meshSize));
        triangles.AddRange(meshData.triangles);

        // Set mesh data to data from newly created mesh
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0);
        if (primitive == MeshPrimitive.Reference)
            mesh.SetNormals(meshData.normals);
        else
            mesh.RecalculateNormals();

        // Upload mesh to graphics API
        mesh.UploadMeshData(false);
        mesh.MarkDynamic();     // Optimizes mesh for frequent manipulation (Unity-related)

        mesh.name = "Softbody Mesh";

        return mesh;
    }

    private void GenerateParticles()
    {
        for (int i = 0; i < meshData.vertices.Length; i++)
        {
            particles.Add(i, new Particle()
            {
                position = transform.position + softMesh.vertices[i],   // Initial position is vertex position
                velocity = Vector3.zero,                                // No initial velocity
                mass = mass / meshData.vertices.Length
            });
        }
    }

    private void GenerateSprings()
    {
        List<string> existingPairs = new List<string>();                    // For checking that particle pairs don't already have a spring connected
        foreach (KeyValuePair<int, Particle> vertexParticle in particles)
        {
            int vIdx = vertexParticle.Key;
            Particle p = vertexParticle.Value;

            // Attach a spring from source vertex particle (vIdx) to all its neighbouring particles
            // i.e. Generate structural springs
            int[] neighbours = meshData.vertexNeighbourSets[vIdx, 0];
            for (int i = 0; i < neighbours.Length; i++)
            {
                string pairID = vIdx.ToString() + neighbours[i].ToString();
                bool pairExists = existingPairs.Contains(pairID);
                bool pairExistsReverse = existingPairs.Contains(pairID.Reverse());
                if (pairExists || pairExistsReverse)
                    continue;
                else
                    existingPairs.Add(vIdx.ToString() + neighbours[i].ToString());

                Particle p1 = particles[vIdx];
                Particle p2 = particles[neighbours[i]];
                Spring spring = new Spring(p1, p2)
                {
                    springConstant = structuralSpringStrength,
                    dampingConstant = structuralSpringDamping,
                    restLength = (p2.position - p1.position).magnitude
                };
                springs.Add(spring);
            }
            
            
            // TODO: Figure out why the below method breaks when using 2 as distance parameter (only when using a custom mesh...?)
            int[] bendingNeighbours = meshData.GetNeighboursAtDistance(vIdx, 2);
            Debug.Log($"Bending neighbour count: {bendingNeighbours.Length}");
            for (int i = 0; i < bendingNeighbours.Length; i++)
            {
                string pairID = vIdx.ToString() + bendingNeighbours[i].ToString();
                bool pairExists = existingPairs.Contains(pairID);
                bool pairExistsReverse = existingPairs.Contains(pairID.Reverse());
                if (pairExists || pairExistsReverse)
                    continue;
                else
                    existingPairs.Add(vIdx.ToString() + bendingNeighbours[i].ToString());

                Particle p1 = particles[vIdx];
                Particle p2 = particles[bendingNeighbours[i]];
                Spring spring = new Spring(p1, p2)
                {
                    springConstant = bendingSpringStrength,
                    dampingConstant = bendingSpringDamping,
                    restLength = (p2.position - p1.position).magnitude
                };
                springs.Add(spring);
            }
            
        }
    }

    private void EulerIntegrationPhysics()
    {
        // Spring forces
        for (int i = 0; i < springs.Count; i++)
        {
            springs[i].Tick();
        }

        // Particle system
        Vector3[] newMeshPositions = new Vector3[softMesh.vertexCount];
        foreach (KeyValuePair<int, Particle> vertexParticle in particles)
        {
            int vIdx = vertexParticle.Key;
            Particle p = vertexParticle.Value;

            // Environmental forces (e.g. gravity, air resistance)
            p.velocity += Vector3.down * gravity * Time.fixedDeltaTime;

            float bounceFactor = 0.5f;
            float frictionFactor = 0.5f;

            // Particle integration
            bool spherecast = Physics.SphereCast(p.position, Particle.SKINWIDTH, p.velocity, out RaycastHit sphereHit, p.velocity.magnitude * Time.fixedDeltaTime);
            if (spherecast)    // Velocity collision
            {
                Vector3 bounceVelocity = Vector3.Reflect(p.velocity, sphereHit.normal) * bounceFactor;
                Vector3 constrainedVelocity = Vector3.ProjectOnPlane(p.velocity, sphereHit.normal) * frictionFactor;
                p.velocity = constrainedVelocity;

                Vector3 toImpactPoint = sphereHit.point - p.position;
                toImpactPoint += sphereHit.normal * Particle.SKINWIDTH;
                p.Move(toImpactPoint);
            }
            else
            {
                Collider[] overlappedColliders = Physics.OverlapSphere(p.position, Particle.SKINWIDTH);
                for (int i = 0; i < overlappedColliders.Length; i++)
                {
                    Vector3 nearPointOnCollider = overlappedColliders[i].ClosestPoint(p.position);
                    Vector3 toSurfacePoint = nearPointOnCollider - p.position;

                    // Velocity
                    Vector3 constrainedVelocity = Vector3.ProjectOnPlane(p.velocity, -toSurfacePoint) * frictionFactor;
                    p.velocity = constrainedVelocity;

                    // Position
                    float distDiff = toSurfacePoint.magnitude - Particle.SKINWIDTH;
                    Vector3 collisionCompensation = toSurfacePoint.normalized * distDiff;

                    if (Physics.Raycast(p.position, collisionCompensation, out RaycastHit compHit, distDiff))
                    {
                        collisionCompensation += compHit.normal * Particle.SKINWIDTH;
                    }

                    p.Move(collisionCompensation);
                }
            }

            p.Move(p.velocity * Time.fixedDeltaTime);

            // New vertex position in mesh
            newMeshPositions[vIdx] = p.position - transform.position;
        }

        // Mesh synchronization
        softMesh.SetVertices(newMeshPositions);
        softMesh.UploadMeshData(false);
    }

    private void VerletIntegrationPhysics()
    {
        // TODO: Verlet integration
    }

    private void OnDrawGizmos()
    {
        if (!softMesh || !showDebug)
            return;

        if (debugFlags.HasFlag(DebugFlag.Vertices))
        {
            Gizmos.color = Color.red;
            for (int i = 0; i < softMesh.vertices.Length; i++)
            {
                Gizmos.DrawSphere(transform.position + softMesh.vertices[i], 0.02f);
#if UNITY_EDITOR
                // UnityEditor.Handles.Label(transform.position + softMesh.vertices[i], "i: " + i);
#endif
            }
        }

        if (debugFlags.HasFlag(DebugFlag.Triangles))
        {
            for (int i = 0; i < softMesh.triangles.Length / 3; i++)
            {
                int i1 = softMesh.triangles[i * 3];
                int i2 = softMesh.triangles[i * 3 + 1];
                int i3 = softMesh.triangles[i * 3 + 2];

                Vector3 avgPos = softMesh.vertices[i1] + softMesh.vertices[i2] + softMesh.vertices[i3];
                avgPos /= 3f;
                avgPos += transform.position;

#if UNITY_EDITOR
                int textSize = Mathf.RoundToInt(16f / (SceneView.currentDrawingSceneView.camera.transform.position - avgPos).magnitude);
                UnityEditor.Handles.Label(avgPos, new GUIContent($"{i1}, {i2}, {i3}"), new GUIStyle() { alignment = TextAnchor.MiddleCenter, fontSize = textSize });
#endif
            }
        }


        if (debugFlags.HasFlag(DebugFlag.Particles))
        {
            for (int i = 0; i < particles.Count; i++)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(particles[i].position, Particle.SKINWIDTH);
                Gizmos.color = Color.red;
                Gizmos.DrawLine(particles[i].position, particles[i].position + particles[i].velocity / 10f);
            }
        }

        if (debugFlags.HasFlag(DebugFlag.Springs))
        {
            for (int i = 0; i < springs.Count; i++)
            {
                Gizmos.color = Color.HSVToRGB((float)i / springs.Count, 1f, 1f);
                Gizmos.DrawLine(springs[i].particle1.position, springs[i].particle2.position);
            }
        }

        if (debugVisualizeVertexIndexInfo < meshData.vertices.Length && debugVisualizeVertexIndexInfo >= 0 && softMesh.vertexCount > 0)
        {
            int[] neighbourList = meshData.vertexNeighbourSets[debugVisualizeVertexIndexInfo, 0];
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(transform.position + softMesh.vertices[debugVisualizeVertexIndexInfo], 0.02f);
            Gizmos.color = Color.blue;
            for (int i = 0; i < neighbourList.Length; i++)
            {
                Gizmos.DrawWireCube(transform.position + softMesh.vertices[neighbourList[i]], Vector3.one * 0.1f);
            }
        }
    }
}
