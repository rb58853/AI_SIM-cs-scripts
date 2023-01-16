using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Agent_Space;
using Triangle_Map;
using BaseNode;

public class randomMove : MonoBehaviour
{
    static System.Random random = new System.Random();
    public NavAgent navAgent { get; private set; }
    public NavMeshAgent a;
    void Start()
    {
        a = GetComponent<NavMeshAgent>();
        a.isStopped = false;
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
        // if (a.destination == null || Vector3.Distance(transform.position, a.destination) <= 0.1f)
        // {
        //     int r = random.Next(0, Environment.map.nodes.Count);
        //     a.SetDestination((Environment.map.nodes[r] as MapNode).triangle.barycenter.ToVector3());
        // }
    }
}
