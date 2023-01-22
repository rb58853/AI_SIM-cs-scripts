using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Point_Map;
using Agent_Space;
using Triangle_Map;

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
            foreach (Triangle_Map.MapNode node in Agent.Metaheuristic.origins[triangle].Keys)
                node.triangle.draw(Color.red);
        }

        if (Input.GetKeyDown(KeyCode.Q))
        {
            foreach (MapNode node in navAgent.agent.ocupedNodes)
            {
                Debug.Log("El agente " + navAgent.agent + " ocupa el triangulo " + node +
                " Lo posee el nodo origin: " + node.origin.agentsIn.Contains(navAgent.agent));
            }
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
