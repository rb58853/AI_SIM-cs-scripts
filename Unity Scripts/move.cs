using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Point_Map;
using Agent_Space;

public class move : MonoBehaviour
{
    public static Triangle_Map.Triangle triangle;
    public NavAgent navAgent { get; private set; }
    public bool noMove;
    void Start()
    {
        navAgent = GetComponent<NavAgent>();
    }
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.W))
        {
            foreach (Triangle_Map.MapNode node in Agent.Metaheuristic.trianglePathToTriangle[triangle])
                node.triangle.draw(Color.red);
        }

        if (noMove) return;
        if (Input.GetMouseButtonUp(1))
        {
            RaycastHit hit;
            if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out hit, 100))
                navAgent.SetDestination(hit.point);
        }

    }
}
