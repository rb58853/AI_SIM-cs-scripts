using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Agent_Space;
using BaseNode;
using System;

public class AgentMove : MonoBehaviour
{
    Agent agent;
    public GameObject sphere;
    public int speed = 15;
    public float water = 1;
    void Start()
    {
        agent = new Agent();
        StartCoroutine(start());
    }
    IEnumerator start()
    {
        yield return new WaitForEndOfFrame();
        yield return new WaitForEndOfFrame();
        agent.setPosition(new Point(transform.position.x, transform.position.y, transform.position.z));
        agent.searchCurrentNode();
        agent.SetPointPath(new Point(sphere.transform.position.x, sphere.transform.position.y, sphere.transform.position.z));
    }

    int countFrames = 0;
    Vector3 dir = new Vector3(0, 0, 0);
    void Update()
    {
        if (!agent.inMove && transform.position.x == agent.position.x && transform.position.z == agent.position.z)
            return;

        countFrames--;
        if (countFrames <= 0)
        {
            agent.NextMove(speed);

            countFrames = 5;
            Vector3 agentPos = new Vector3(agent.position.x, agent.position.y, agent.position.z);
            dir = (agentPos - transform.position) / 5;
        }

        Debug.Log("Desplazamiento del vector: " + Mathf.Sqrt((dir.x * dir.x) + (dir.y * dir.y) + (dir.z * dir.z)));
        transform.Translate(dir);
    }

    public void SetDestination(Vector3 destination)
    {
        agent.SetPointPath(new Point(destination.x, destination.y, destination.z));
    }
}
