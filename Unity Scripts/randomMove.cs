using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Agent_Space;
using Triangle_Map;

public class randomMove : MonoBehaviour
{
    static System.Random random  = new System.Random();
    public NavAgent navAgent { get; private set; }
    void Start()
    {
        navAgent = GetComponent<NavAgent>();
    }
    void Update()
    {
        if (!navAgent.InMove())
        {
            int r = random.Next(0, Environment.map.nodes.Count);
            navAgent.SetDestination((Environment.map.nodes[r] as MapNode).triangle.barycenter);
        }
    }
}
