using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Triangle_Map;
using System;
using Point_Map;
using Agent_Space;
using BaseNode;

public class GetTriangles : MonoBehaviour
{
    static List<Vector3[]> Triangles;
    static Dictionary<Vector3, Dictionary<Vector3, List<Vector3[]>>> triangleFromArist;
    static Dictionary<Vector3[], MapNode> nodeFromTriangle;
    static Dictionary<MapNode, Vector3[]> triangleFromNode;
    public void Start()
    {
        GetTrianglesFromNavMesh();
        SetMapWithTriangles();
        DFSDraw(Agent.map.nodes[80] as MapNode);
    }

    void DFSDraw(MapNode node)
    {
        DrawByTriangle(node.triangle);
        node.SetVisited();
        foreach (MapNode adj in node.adjacents.Keys)
            if (!adj.visited)
                DFSDraw(adj);
    }
    void GetTrianglesFromNavMesh()
    {
        triangleFromArist = new Dictionary<Vector3, Dictionary<Vector3, List<Vector3[]>>>();
        Triangles = new List<Vector3[]>();
        NavMeshTriangulation navMesh = NavMesh.CalculateTriangulation();
        Vector3[] vertices = navMesh.vertices;
        int[] polygons = navMesh.indices;

        for (int i = 0; i < polygons.Length; i += 3)
        {
            Vector3 v1 = vertices[polygons[i]];
            Vector3 v2 = vertices[polygons[i + 1]];
            Vector3 v3 = vertices[polygons[i + 2]];
            Vector3[] triangle = new Vector3[] { v1, v2, v3 };
            Triangles.Add(triangle);
            AddArist(v1, v2, triangle);
            AddArist(v2, v3, triangle);
            AddArist(v3, v1, triangle);
        }

    }
    void AddArist(Vector3 v1, Vector3 v2, Vector3[] triangle)
    {
        ///if v1 dont' exist then v2 does not exist either, because the refelxive relation in arists

        if (triangleFromArist.ContainsKey(v1))
        {
            if (triangleFromArist[v1].ContainsKey(v2))
            {
                ///Add to <v1,v2> and <v2,v1>, because the value(list) is the same reference
                triangleFromArist[v1][v2].Add(triangle);
                if (!triangleFromArist[v2][v1].Contains(triangle))///Estas 2 lineas(v) realmente sobra 
                    triangleFromArist[v2][v1].Add(triangle);
            }

            else
            {
                ///Add to <v1,v2> and <v2,v1> the same reference of list, reflexive relation in arists
                List<Vector3[]> list = new List<Vector3[]>(); list.Add(triangle);
                triangleFromArist[v1].Add(v2, list);

                if (triangleFromArist.ContainsKey(v2))
                    triangleFromArist[v2].Add(v1, list);
                else
                {
                    Dictionary<Vector3, List<Vector3[]>> dict = new Dictionary<Vector3, List<Vector3[]>>();
                    dict.Add(v1, list);
                    triangleFromArist.Add(v2, dict);
                }
            }
        }
        else
        {
            ///Add to <v1,v2> and <v2,v1> the same reference of list, reflexive relation in arists
            List<Vector3[]> list = new List<Vector3[]>(); list.Add(triangle);

            if (!triangleFromArist.ContainsKey(v2))
            {
                Dictionary<Vector3, List<Vector3[]>> dict1 = new Dictionary<Vector3, List<Vector3[]>>();
                dict1.Add(v1, list);
                triangleFromArist.Add(v2, dict1);
            }
            else
                triangleFromArist[v2].Add(v1, list);

            Dictionary<Vector3, List<Vector3[]>> dict2 = new Dictionary<Vector3, List<Vector3[]>>();
            dict2.Add(v2, list);
            triangleFromArist.Add(v1, dict2);
        }
    }
    void DrawTriangles(List<Vector3[]> triangles = null)
    {
        if (triangles == null)
            triangles = Triangles;

        foreach (Vector3[] triangle in triangles)
            DrawOnlyTriangle(triangle);
    }
    void DrawOnlyTriangle(Vector3[] triangle)
    {
        Debug.DrawLine(triangle[0], triangle[1], Color.red, 50f);
        Debug.DrawLine(triangle[1], triangle[2], Color.red, 50f);
        Debug.DrawLine(triangle[2], triangle[0], Color.red, 50f);
    }
    void DrawByTriangle(Triangle triangle)
    {
        Vector3 p1 = new Vector3(triangle.vertex1.x, triangle.vertex1.y, triangle.vertex1.z);
        Vector3 p2 = new Vector3(triangle.vertex2.x, triangle.vertex2.y, triangle.vertex2.z);
        Vector3 p3 = new Vector3(triangle.vertex3.x, triangle.vertex3.y, triangle.vertex3.z);

        Debug.DrawLine(p1, p2, Color.red, 50f);
        Debug.DrawLine(p2, p3, Color.red, 50f);
        Debug.DrawLine(p3, p1, Color.red, 50f);
    }
    void SetMapWithTriangles()
    {
        nodeFromTriangle = new Dictionary<Vector3[], MapNode>();
        triangleFromNode = new Dictionary<MapNode, Vector3[]>();

        Agent.NewMap();
        Map map = Agent.map;

        foreach (Vector3[] triangle in Triangles)
        {
            Point p1 = new Point(triangle[0].x, triangle[0].y, triangle[0].z);
            Point p2 = new Point(triangle[1].x, triangle[1].y, triangle[1].z);
            Point p3 = new Point(triangle[2].x, triangle[2].y, triangle[2].z);

            MapNode node = new MapNode(new Triangle(p1, p2, p3));
            nodeFromTriangle.Add(triangle, node);
            triangleFromNode.Add(node, triangle);

            map.AddNode(node);
        }
        foreach (MapNode node in map.nodes)
        {
            foreach (Tuple<Vector3[], Arist> adjVector in GetAdjacents(triangleFromNode[node]))
            {
                MapNode adjNode = nodeFromTriangle[adjVector.Item1];

                if (!node.adjacents.ContainsKey(adjNode))
                    node.AddAdjacent(adjNode, adjVector.Item2);
                if (!adjNode.adjacents.ContainsKey(node))
                    adjNode.AddAdjacent(node, adjVector.Item2);
            }
        }
    }
    List<Tuple<Vector3[], Arist>> GetAdjacents(Vector3[] triangle)
    {
        List<Tuple<Vector3[], Arist>> result = new List<Tuple<Vector3[], Arist>>();

        Vector3[] e1 = new Vector3[] { triangle[0], triangle[1] };
        Vector3[] e2 = new Vector3[] { triangle[1], triangle[2] };
        Vector3[] e3 = new Vector3[] { triangle[2], triangle[0] };

        foreach (Vector3[] polygon in triangleFromArist[e1[0]][e1[1]])
            if (polygon != triangle)
                result.Add(new Tuple<Vector3[], Arist>(polygon, eToArist(e1)));

        foreach (Vector3[] polygon in triangleFromArist[e2[0]][e2[1]])
            if (polygon != triangle)
                result.Add(new Tuple<Vector3[], Arist>(polygon, eToArist(e2)));

        foreach (Vector3[] polygon in triangleFromArist[e3[0]][e3[1]])
            if (polygon != triangle)
                result.Add(new Tuple<Vector3[], Arist>(polygon, eToArist(e3)));

        return result;
    }
    Arist eToArist(Vector3[] e)
    {
        float x = e[0][0]; float y = e[0][1]; float z = e[0][2];
        Point p1 = new Point(x, y, z);
        x = e[1][0]; y = e[1][1]; z = e[1][2];
        Point p2 = new Point(x, y, z);

        return new Arist(p1, p2);
    }
}