using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class TetrahedronVisualizer : MonoBehaviour
{
    [Header("Settings")]
    public Vector3[] tetrahedronPoints = new Vector3[] { Vector3.up, Vector3.right, Quaternion.AngleAxis(360f / 3f, Vector3.up) * Vector3.right, Quaternion.AngleAxis(2f * 360f / 3f, Vector3.up) * Vector3.right };
    public Vector3 localCartesianPoint = Vector3.one * 0.5f;
    public Vector4 localBayesianPoint = new Vector4(1f, 0f, 0f, 0f);

    private void OnValidate()
    {
        localBayesianPoint.x = Mathf.Clamp01(localBayesianPoint.x);
        localBayesianPoint.y = Mathf.Clamp01(localBayesianPoint.y);
        localBayesianPoint.z = Mathf.Clamp01(localBayesianPoint.z);
        localBayesianPoint.w = Mathf.Clamp01(localBayesianPoint.w);
    }

    private void OnDrawGizmos()
    {
        for (int i = 0; i < 4; i++)
        {
            // Draw triangle
            Handles.color = PointInsideTetrahedron(localCartesianPoint) ? Color.green : new Color(1f, 0f, 0f, 0.5f);
            int i0 = ((byte)(i + 1)).GetBit(3) ? 1 : 0;
            int i1 = 1 + (i + 0) % 3;
            int i2 = 1 + (i + 1) % 3;

            Vector3[] triangle = new Vector3[] {
                transform.position + tetrahedronPoints[i0],
                transform.position + tetrahedronPoints[i1],
                transform.position + tetrahedronPoints[i2]
            };

            Handles.DrawAAConvexPolygon(triangle);

            // Draw all edges
            Handles.color = new Color(0f, 0f, 1f, 0.5f);
            for (int j = 0; j < triangle.Length; j++)
            {
                int v0 = j;
                int v1 = (j + 1) % triangle.Length;
                Handles.DrawLine(triangle[v0], triangle[v1], 2f);
            }
        }

        // Draw barycentric point
        Gizmos.color = new Color(0f, 1f, 0f, 0.5f);
        Vector3 point = transform.position + localCartesianPoint;
        Gizmos.DrawSphere(point, 0.05f);
    }

    // Local space
    private Vector3 BarycentricToCartesian(Vector4 baryPoint)
    {
        Vector3 cartPos = Vector3.zero;

        return cartPos;
    }

    private bool PointInsideTetrahedron(Vector3 point)
    {
        // Each vertex's implicit barycentric coordinates
        // v0 = w(1, 0, 0, 0)
        // v1 = w(0, 1, 0, 0)
        // v2 = w(0, 0, 1, 0)
        // v3 = w(0, 0, 0, 1)

        // Barycentric coordinates for point in a tetrahedron:
        // (w1, w2, w3) = T^-1 * (p - v3)
        // T =  [ x1 - x4, x2 - x4, x3 - x4 ]
        //      [ y1 - y4, y2 - y4, y3 - y4 ]
        //      [ z1 - z4, z2 - z4, z3 - z4 ]

        // Points
        Vector4 p = new Vector4(point.x, point.y, point.z, 1f);
        Vector4 a = new Vector4(tetrahedronPoints[0].x, tetrahedronPoints[0].y, tetrahedronPoints[0].z, 1f);
        Vector4 b = new Vector4(tetrahedronPoints[1].x, tetrahedronPoints[1].y, tetrahedronPoints[1].z, 1f);
        Vector4 c = new Vector4(tetrahedronPoints[2].x, tetrahedronPoints[2].y, tetrahedronPoints[2].z, 1f);
        Vector4 d = new Vector4(tetrahedronPoints[3].x, tetrahedronPoints[3].y, tetrahedronPoints[3].z, 1f);

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
        
        return u >= 0f && v >= 0f && w >= 0f && x >= 0f;
    }
}
