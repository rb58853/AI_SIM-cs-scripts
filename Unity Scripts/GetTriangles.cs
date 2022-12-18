using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class GetTriangles : MonoBehaviour
{
    public static List<Vector3[]> Triangles { get; private set; }
    public void Start()
    {
        GetTrianglesFromNavMesh();
        PrintTriangles();
    }
    public void GetTrianglesFromNavMesh()
    {
        Triangles = new List<Vector3[]>();
        NavMeshTriangulation navMesh = NavMesh.CalculateTriangulation();
        Vector3[] vertices = navMesh.vertices;
        int[] polygons = navMesh.indices;


        for (int i = 0; i < polygons.Length; i += 3)
        {
            Vector3 v1 = vertices[polygons[i]];
            Vector3 v2 = vertices[polygons[i + 1]];
            Vector3 v3 = vertices[polygons[i + 2]];
            Triangles.Add(new Vector3[] { v1, v2, v3 });
        }
    }
    public void PrintTriangles()
    {
        foreach (Vector3[] triangle in Triangles)
        {
            Debug.DrawLine(triangle[0], triangle[1], Color.red, 50f);
            Debug.DrawLine(triangle[1], triangle[2], Color.red, 50f);
            Debug.DrawLine(triangle[2], triangle[0], Color.red, 50f);
        }
    }
    public void SetMapWithTriangles()
    {
        
    }
}
