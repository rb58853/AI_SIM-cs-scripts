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
    public float water = 1;
    public float fire = 1;
    void Start()
    {
        agent = new Agent(radius, name);
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
        {
            countFrames = 0;
            return;
        }
        countFrames--;
        // agent.NextMove(3);

        if (countFrames <= 0)
        {
            agent.NextMove(speed);

            countFrames = 5;
            // Vector3 agentPos = new Vector3(agent.position.x, agent.position.y, agent.position.z);
            // dir = (agentPos - transform.position) / 10;


            Point agentPosition = agent.position;
            if (Vector3.Distance(agentPosition.ToVector3(), transform.position) > 1)
            {
                PointNode.Static.DrawTwoPoints(agentPosition,
                new Point(transform.position.x, transform.position.y, transform.position.z), Color.red);
            }
            transform.position = new Vector3(x: agentPosition.x, y: transform.position.y, z: agentPosition.z);

            Point nextPosition = agent.nextPosition.point;
            Point position = agent.currentPosition.point;

            dir = ((((nextPosition - agentPosition) * speed) / (agentPosition.Distance(nextPosition) * 25f)) / 5f).ToVector3();
        }
        transform.Translate(dir);

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
