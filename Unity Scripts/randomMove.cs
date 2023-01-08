using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Agent_Space;
using Triangle_Map;

public class randomMove : MonoBehaviour
{
    static System.Random random = new System.Random();
    public NavAgent navAgent { get; private set; }
    void Start()
    {
        navAgent = GetComponent<NavAgent>();
    }
    int o = 0;
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.E))
        {
            // Debug.Log("Press e");
            // navAgent.agent.inMove = false;
        }
        if (!navAgent.InMove())
        {
            int r = random.Next(0, Environment.map.nodes.Count);
            navAgent.SetDestination((Environment.map.nodes[r] as MapNode).triangle.barycenter);
        }
    }
}
