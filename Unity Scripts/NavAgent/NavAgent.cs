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
    public bool grupalMove = false;
    void Start()
    {
        agent = new Agent(radius, name);
        if (grupalMove)
            agent.setGrup();
            
        GetTriangles.Start();
        agent.setPosition(new Point(transform.position.x, transform.position.y, transform.position.z));
        agent.searchCurrentNode();
    }
    int countFrames = 0;
    int framesForUpdatePath = 0;
    Vector3 dir = new Vector3(0, 0, 0);
    bool inMove = true;
    void Update()
    {


        if (!agent.inMove && transform.position.x == agent.position.x && transform.position.z == agent.position.z)
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
