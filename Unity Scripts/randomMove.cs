using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Agent_Space;
using Triangle_Map;

public class randomMove : MonoBehaviour
{
    static System.Random random = new System.Random();
    public NavAgent navAgent { get; private set; }
    public bool patrulla = false;
    void Start()
    {
        navAgent = GetComponent<NavAgent>();
    }
    int o = 0;
    void Update()
    {

        if (!navAgent.InMove())
        {
            if (patrulla)
            {
                if (o == 0)
                    o = Environment.map.nodes.Count - 1;
                else
                    o = 0;

                navAgent.SetDestination((Environment.map.nodes[o] as MapNode).triangle.barycenter);
                return;
            }

            int r = random.Next(0, Environment.map.nodes.Count);
            navAgent.SetDestination((Environment.map.nodes[r] as MapNode).triangle.barycenter);
        }
    }
}
