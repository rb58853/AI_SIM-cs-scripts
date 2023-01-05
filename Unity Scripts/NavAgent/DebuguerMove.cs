using Agent_Space;
using BaseNode;
using Point_Map;
using System.Collections;
using System.Collections.Generic;
using Triangle_Map;
using UnityEngine;

public class DebuguerMove : MonoBehaviour
{
    Vector3 endPosition;
    Agent agent;

    void Start()
    {
        StartCoroutine(start());
    }
    IEnumerator start()
    {
        yield return new WaitForEndOfFrame();
        agent = GetComponent<move>().navAgent.agent;

        //agent.setPosition(new Point(transform.position.x, transform.position.y, transform.position.z));
        //agent.searchCurrentNode();
        //DFSDraw(agent.currentNode, Color.yellow);

        endPosition = new Vector3(1, 1, 1);
    }
    // Update is called once per frame

    Node[] tempT = new Node[] { };
    PointNode[] points = new PointNode[] { };


    void Update()
    {
        if (agent == null) return;
        if (Input.GetKeyDown(KeyCode.E))
        {
            Debug.Log("El material del nodo actual es: " + agent.currentNode.material);
        }
        if (Input.GetMouseButton(1))
        {
            DrawPath(tempT, Color.clear);
            DrawPath(points, Color.white);

            RaycastHit hit;
            if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out hit, 100))
            {

                Point end = new Point(hit.point.x, hit.point.y, hit.point.z);
                tempT = agent.GetTrianglePath(end);
                if (tempT != null)
                {
                    DrawPath(tempT, Color.black);
                    points = agent.GetPointPath(end);
                    DrawPath(points, Color.blue);
                }
            }
        }
    }
    void DrawPath(PointNode[] points, Color color)
    {
        for (int i = 0; i < points.Length - 1; i++)
        {
            Vector3 p1 = new Vector3(points[i].get_x(), points[i].get_y(), points[i].get_z());
            Vector3 p2 = new Vector3(points[i + 1].get_x(), points[i + 1].get_y(), points[i + 1].get_z());
            Debug.DrawLine(p1, p2, color, 50f);
        }
    }
    void DrawPath(Node[] nodes, Color color)
    {
        foreach (Node node in nodes)
            DrawFromTriangle((node as MapNode).triangle, color);
    }
    void DrawFromTriangle(Triangle triangle, Color color)
    {
        Vector3 p1 = new Vector3(triangle.vertex1.x, triangle.vertex1.y, triangle.vertex1.z);
        Vector3 p2 = new Vector3(triangle.vertex2.x, triangle.vertex2.y, triangle.vertex2.z);
        Vector3 p3 = new Vector3(triangle.vertex3.x, triangle.vertex3.y, triangle.vertex3.z);

        Debug.DrawLine(p1, p2, color, 50f);
        Debug.DrawLine(p2, p3, color, 50f);
        Debug.DrawLine(p3, p1, color, 50f);
    }
    void DFSDraw(MapNode node, Color color)
    {
        DrawFromTriangle(node.triangle, color);
        node.SetVisited();
        foreach (MapNode adj in node.adjacents.Keys)
            if (!adj.visited)
                DFSDraw(adj, color);
    }

}
