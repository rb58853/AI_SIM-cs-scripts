using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using Agent_Space;
using BaseNode;
using System;

public class NavAgent : MonoBehaviour
{
    //public string Name = "agent";
    public float radius = 1;
    public Agent agent { get; private set; }
    public int speed = 15;
    public float water = 1;
    public float fire = 1;
    void Start()
    {
        agent = new Agent(radius,name);
        GetTriangles.Start();
        agent.setPosition(new Point(transform.position.x, transform.position.y, transform.position.z));
        agent.searchCurrentNode();
    }
    int countFrames = 0;
    int framesForUpdatePath = 0;
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
        transform.Translate(dir);

        if (framesForUpdatePath <= 0)
        {
            framesForUpdatePath = 5;
            //agent.SetPointPath(agent.destination);
        }
        framesForUpdatePath--;


    }
    public void SetDestination(Vector3 destination)
    {
        agent.SetPointPath(new Point(destination.x, destination.y, destination.z));
    }
    public void SetDestination(Point destination)
    {
        agent.SetPointPath(destination);
    }

    public bool InMove()
    {
        return agent.inMove;
    }
}
