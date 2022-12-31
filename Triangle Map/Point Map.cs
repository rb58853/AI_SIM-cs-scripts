﻿using System.Collections.Generic;
using Triangle_Map;
using BaseNode;
using DijkstraSpace;
using UnityEngine;
using Agent_Space;
using System;

namespace Point_Map
{
    public class PointNode : Node
    {

        PointNode end;
        public Point point { get; private set; }
        public Dictionary<PointNode, float> adjacents { get; private set; }
        public bool visitedInCreation = false;

        public PointNode(Point point, PointNode end = null)
        {
            this.point = point;
            adjacents = new Dictionary<PointNode, float>();
            distance = float.MaxValue;
            this.end = end;
        }
        public float get_x() { return point.x; }
        public float get_y() { return point.y; }
        public float get_z() { return point.z; }

        public void SetEnd(PointNode node) { end = node; }
        public void AddAdjacent(PointNode node, float value = 1) { adjacents.Add(node, value); }
        public float EuclideanDistance(PointNode node) { return point.Distance(node.point); }
        float Heuristic(PointNode endNode) { return EuclideanDistance(endNode); }
        public override float Value()
        {
            float g = distance;
            float h = Heuristic(end) * Agent_Space.Environment.heuristicPointWeigth;

            return g + h;
        }
        public int CompareTo(PointNode other)
        {
            return this.value.CompareTo(other.value);
        }
        public override string ToString()
        {
            return point.ToString();
        }

        public override List<Node> GetAdyacents()
        {
            /// Esto en cuanto a eficiencia es malo, se esta sacrificando en eficiencia para ganar en genaricidad
            /// Cambiar el uso de lista por el uso de array en caso que deje usar el .toArray que no se cuanto cuesta tampoco
            List<Node> result = new List<Node>();
            foreach (Node adj in adjacents.Keys)
                result.Add(adj);
            return result;
        }

        public override float Distance(Node node)
        {
            return EuclideanDistance(node as PointNode);// * adjacents[node as PointNode];
        }

        internal class Static
        {
            public static List<PointNode> CreatePointMap(List<Arist> arists, Point init, Point end, Agent agent = null, float n = 1, float cost = 1)
            {
                List<PointNode> result = new List<PointNode>();
                MapNode[] mapNodes = agent.trianglePath.ToArray();

                if (arists.Count == 0)
                {
                    PointNode e = new PointNode(end); PointNode i = new PointNode(init, e);
                    e.SetEnd(e);
                    result.Add(i);
                    i.visitedInCreation = true;

                    List<PointNode> temp = new List<PointNode>(); temp.Add(e);
                    CreateSimplePath(i, temp, agent, agent.currentNode, agent.currentNode.MaterialCost(agent), e, result);

                    result.Add(e);
                    return result;
                }

                List<List<PointNode>> points = new List<List<PointNode>>();
                PointNode endNode = new PointNode(end); endNode.SetEnd(endNode);
                PointNode initNode = new PointNode(init, endNode);
                initNode.visitedInCreation = true;

                foreach (Arist arist in arists)
                {
                    points.Add(new List<PointNode>());
                    foreach (Point point in arist.ToPoints(n))
                        points[points.Count - 1].Add(new PointNode(point, endNode));
                }

                CreateSimplePath(initNode, points[0], agent, agent.currentNode, cost, endNode, result);

                for (int i = 0; i < points.Count - 1; i++)
                    for (int j = 0; j < points[i].Count; j++)
                        CreateSimplePath(points[i][j], points[i + 1], agent, mapNodes[i + 1].origin, arists[i].materialCost, endNode, result);

                foreach (PointNode node in points[points.Count - 1])
                {
                    List<PointNode> temp = new List<PointNode>(); temp.Add(endNode);
                    CreateSimplePath(node, temp, agent, mapNodes[points.Count].origin, arists[points.Count - 1].materialCost, endNode, result);
                }

                result.Add(endNode);
                return result;
            }
            public static Tuple<bool, Agent> Collision(Point node1, Point node2, Agent agent, MapNode mapNode)
            {
                Point l1 = node1;
                Point l2 = node2;

                foreach (Agent agentObstacle in mapNode.agentsIn)
                {
                    if (agentObstacle == agent) continue;

                    if (agentObstacle.position.DistanceToSegment(l1, l2) < agent.radius + agentObstacle.radius)
                        /// Collision
                        return new Tuple<bool, Agent>(true, agentObstacle);
                }
                return new Tuple<bool, Agent>(false, null);
            }

            static void CreateSimplePath(PointNode init, List<PointNode> list,
                Agent agent, MapNode mapNode, float cost, PointNode endNode,
                List<PointNode> result)
            {

                if (!init.visitedInCreation) return;

                List<PointNode> endList = new List<PointNode>();
                endList.Add(init);

                result.Add(init);
                init.SetDistance(0);
                HeapNode q = new HeapNode(init);


                List<Agent> visitedObstacles = new List<Agent>();

                //int overflow = 0;

                List<PointNode> temp = new List<PointNode>();
                foreach (PointNode node in list)
                    temp.Add(node);

                while (temp.Count > 0 && q.size > 0)
                {
                    //overflow++;
                    //Debug.Log("Overflow: " + overflow);

                    PointNode current = q.Pop() as PointNode;

                    for (int i = 0; i < temp.Count; i++)
                    {
                        PointNode end = temp[i];

                        Tuple<bool, Agent> collision = Collision(current.point, end.point, agent, mapNode);
                        if (collision.Item1)
                        {
                            if (!visitedObstacles.Contains(collision.Item2))
                            {
                                visitedObstacles.Add(collision.Item2);

                                PointNode another1 = new PointNode(GeneratedPoint(init.point, agent, collision.Item2), endNode);
                                if (mapNode.triangle.PointIn(another1.point))
                                    if (!Collision(current.point, another1.point, agent, mapNode).Item1)
                                    {
                                        current.AddAdjacent(another1, cost);
                                        //DrawTwoPoints(current.point, another1.point);
                                        another1.visitedInCreation = true;
                                        another1.SetDistance(current.distance + current.EuclideanDistance(another1));
                                        q.Push(another1);
                                        result.Add(another1);
                                        endList.Add(another1);
                                    }
                                PointNode another2 = new PointNode(GeneratedPoint(init.point, agent, collision.Item2, true), endNode);
                                if (mapNode.triangle.PointIn(another2.point))
                                    if (!Collision(current.point, another2.point, agent, mapNode).Item1)
                                    {
                                        current.AddAdjacent(another2, cost);
                                        //DrawTwoPoints(current.point, another2.point);
                                        another2.visitedInCreation = true;
                                        another2.SetDistance(current.distance + current.EuclideanDistance(another2));
                                        q.Push(another2);
                                        result.Add(another2);
                                        endList.Add(another2);
                                    }
                            }
                            continue;
                        }

                        current.AddAdjacent(end, cost);
                        //DrawTwoPoints(current.point, end.point);
                        end.visitedInCreation = true;
                        temp.Remove(end); i--;
                    }
                    foreach (PointNode node in endList)
                    {
                        ///Resetear la distancia en todos los nodos que me interesan, solo me interesa los init y los nodos extras que se agregaron
                        ///xq los end son init en otra entrada.
                        node.SetDistance(float.MaxValue);
                    }
                    ///Unico end que no sera nunk un init, si funciona bien se pone arriba
                    endNode.SetDistance(float.MaxValue);
                }

            }
            static Point GeneratedPoint(Point init, Agent agent, Agent collision, bool negative = false, float n = 2.2f)
            {
                Point vector1 = collision.position - init;
                Point vector2 = new Point(-vector1.z, 0, vector1.x);///Ortogonal
                float den = (float)Math.Sqrt(vector2.x * vector2.x + vector2.z * vector2.z);
                Point unitVector2 = vector2 / den;

                if (negative)
                    return collision.position + unitVector2 * (agent.radius + collision.radius) * -n;
                return collision.position + unitVector2 * (agent.radius + collision.radius) * n;
            }
            static void DrawTwoPoints(Point p1, Point p2)
            {
                Vector3 a = new Vector3(p1.x, p1.y, p1.z);
                Vector3 b = new Vector3(p2.x, p2.y, p2.z);
                Debug.DrawLine(a, b, Color.black, 50f);
            }
        }
    }

}
