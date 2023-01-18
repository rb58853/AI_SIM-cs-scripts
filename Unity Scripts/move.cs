using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Point_Map;
using Agent_Space;

public class move : MonoBehaviour
{
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
            // transform.position = agent.position.ToVector3();
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
