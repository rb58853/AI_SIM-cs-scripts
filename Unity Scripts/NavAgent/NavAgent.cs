using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Agent_Space;
using BaseNode;
using Point_Map;
using System;

public class NavAgent : MonoBehaviour
{
    public float radius = 1;
    public Agent agent { get; private set; }
    public int speed = 15;
    public Agent_Space.AgentType agentType = Agent_Space.AgentType.normal;
    public bool grupalMove = false;
    void Start()
    {
        agent = new Agent(radius, name,agentType);
        if (grupalMove)
            agent.setGrup();

        GetTriangles.Start();
        agent.setPosition(new Point(transform.position.x, transform.position.y, transform.position.z));
    }
    int countFrames = 0;
    int framesForUpdatePath = 0;
    Vector3 dir = new Vector3(0, 0, 0);
    bool inMove = true;
    void Update()
    {
        if (!agent.inMove && Vector3.Distance(transform.position, agent.position.ToVector3()) < 0.1f)
        {
            countFrames = 0;
            return;
        }

        countFrames--;

        if (countFrames <= 0)
        {
            countFrames = 5;
            agent.NextMove(speed);
            Vector3 position = new Vector3(agent.position.x,
            transform.position.y + agent.position.ToVector3().y,
            agent.position.z);
            dir = (position - transform.position) / 5;
        }
        transform.Translate(dir);
    }
    System.Random r = new System.Random();
    IEnumerator setDestination(Point destination)
    {
        // int len = r.Next(0, 50);
        int len = agent.posInGrup / 1;
        // int minDistance = 100;
        // int minDistance = int.MaxValue;
        // int maxDistance = 100;
        // agent.inMove = false;
        // for (int i = 0; i < 100; i++)
        // {
        //     Agent temp = Agent_Space.Environment.Interactive.grup[i];
        //     minDistance = Math.Min(minDistance, (int)temp.position.Distance(destination));
        // }

        // int len = (int)agent.position.Distance(destination) - minDistance;
        // int len = maxDistance - (int)agent.position.Distance(destination);
        for (int i = 0; i < len; i++)
            yield return new WaitForEndOfFrame();
        agent.SetPointPath(destination);
    }
    public void SetDestination(Vector3 destination)
    {
        SetDestination(new Point(destination.x, destination.y, destination.z));
    }
    public void SetDestination(Point destination)
    {
        if (grupalMove)
        {
            // StopCoroutine(setDestination(destination));
            StartCoroutine(setDestination(destination));
        }
        else
            agent.SetPointPath(destination);
    }

    public bool InMove()
    {
        return agent.inMove;
    }
}
