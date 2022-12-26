using Agent_Space;
using BaseNode;
using Point_Map;
using System.Collections;
using System.Collections.Generic;
using Triangle_Map;
using UnityEngine;

public class Move : MonoBehaviour
{
    public GameObject sphere;
    Vector3 position;
    Agent agent;

    void Start()
    {
        StartCoroutine(start());
    }
    IEnumerator start()
    {
        yield return new WaitForEndOfFrame();
        agent = new Agent();
        agent.setCurrentNode(Agent.map.nodes[80] as MapNode);
        agent.setPosition((Agent.map.nodes[80] as MapNode).triangle.barycenter);
        position = new Vector3(1, 1, 1);

    }
    // Update is called once per frame
    void Update()
    {
        if (position != sphere.transform.position)
        {
            position = sphere.transform.position;
            Point end = new Point(sphere.transform.position.x, sphere.transform.position.y, sphere.transform.position.z);
            PointNode[] points = agent.GetPointPath(end);
            DrawPath(points);
        }
    }
    void DrawPath(PointNode[] points)
    {
        for (int i = 0; i < points.Length - 1; i++)
        {
            Vector3 p1 = new Vector3(points[i].get_x(), points[i].get_y(), points[i].get_z());
            Vector3 p2 = new Vector3(points[i + 1].get_x(), points[i + 1].get_y(), points[i + 1].get_z());
            Debug.DrawLine(p1, p2, Color.blue, 50f);
        }
    }

}
