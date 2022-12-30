using System.Collections.Generic;
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
            float h = Heuristic(end);

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
                    PointNode e = new PointNode(end);
                    PointNode i = new PointNode(init, e);
                    e.SetEnd(e);
                    result.Add(i);
                    i.visitedInCreation = true;
                    //i.AddAdjacent(e, cost);

                    List<PointNode> temp = new List<PointNode>(); temp.Add(e);
                    CreateSimplePath(i, temp, agent, agent.currentNode, agent.currentNode.MaterialCost(agent), e, result, new List<Agent>());

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

                CreateSimplePath(initNode, points[0], agent, agent.currentNode, cost, endNode, result, new List<Agent>());

                for (int i = 0; i < points.Count - 1; i++)
                    for (int j = 0; j < points[i].Count; j++)
                        CreateSimplePath(points[i][j], points[i + 1], agent, mapNodes[i + 1].origin, arists[i].materialCost, endNode, result, new List<Agent>());

                foreach (PointNode node in points[points.Count - 1])
                {
                    List<PointNode> temp = new List<PointNode>(); temp.Add(endNode);
                    CreateSimplePath(node, temp, agent, mapNodes[points.Count].origin, arists[points.Count - 1].materialCost, endNode, result, new List<Agent>());
                }

                result.Add(endNode);
                return result;
            }
            static Tuple<bool, Agent> Collision(PointNode node1, PointNode node2, Agent agent, MapNode mapNode)
            {
                Point l1 = node1.point;
                Point l2 = node2.point;

                foreach (Agent agentObstacle in mapNode.agentsIn)
                {
                    float lenSegment = l1.Distance(l2);
                    Point intersected = IntersectedOrtogonalVectors(l1, l2, agentObstacle.position);
                    bool inSegment = intersected.Distance(l1) < lenSegment && intersected.Distance(l2) < lenSegment;

                    if (agentObstacle == agent) continue;

                    //if (agentObstacle.position.DistanceToLine(l1, l2) < agent.radius + agentObstacle.radius)
                    if (inSegment)
                        if (intersected.Distance(agentObstacle.position) < agent.radius + agentObstacle.radius)
                            /// Collision
                            return new Tuple<bool, Agent>(true, agentObstacle);
                }

                return new Tuple<bool, Agent>(false, null);
            }

            static void CreateSimplePath(PointNode init, List<PointNode> list,
                Agent agent, MapNode mapNode, float cost, PointNode endNode,
                List<PointNode> result, List<Agent> visitedObstacles)
            {

                if (!init.visitedInCreation) return;
                PointNode before = null;
                int index = 0;

                result.Add(init);
                foreach (PointNode end in list)
                {
                    Tuple<bool, Agent> collision = Collision(init, end, agent, mapNode);
                    if (collision.Item1)
                    {
                        if (visitedObstacles.Contains(collision.Item2))
                            continue;

                        PointNode another1 = null;
                        PointNode another2 = null;
                        //if (before != null && !visitedObstacles.Contains(collision.Item2))
                        //  another = new PointNode(IntersectedOrtogonalVectors(init.point, before.point, collision.Item2.position), endNode);
                        //else
                        another1 = new PointNode(GeneratedPoint(init.point, agent, collision.Item2), endNode);
                        another2 = new PointNode(GeneratedPoint(init.point, agent, collision.Item2, negative: true), endNode);
                        //Debug.Log(another);

                        if (mapNode.triangle.PointIn(another1.point))
                            if (!Collision(init, another1, agent, mapNode).Item1)
                            {
                                init.AddAdjacent(another1, cost);
                                result.Add(another1);
                                another1.visitedInCreation = true;
                                visitedObstacles.Add(collision.Item2);
                                CreateSimplePath(another1, list.GetRange(index, list.Count - index), agent, mapNode, cost, endNode, result, visitedObstacles);
                                //visitedObstacles.Remove(collision.Item2); ///More path, but very much complex
                            }

                        if (mapNode.triangle.PointIn(another2.point))
                            if (!Collision(init, another2, agent, mapNode).Item1)
                            {
                                init.AddAdjacent(another2, cost);
                                result.Add(another2);
                                another2.visitedInCreation = true;
                                visitedObstacles.Add(collision.Item2);
                                CreateSimplePath(another2, list.GetRange(index, list.Count - index), agent, mapNode, cost, endNode, result, visitedObstacles);
                                //visitedObstacles.Remove(collision.Item2); ///More path, but very much complex
                            }

                        continue;
                    }

                    Vector3 a = new Vector3(init.point.x, init.point.y, init.point.z);
                    Vector3 b = new Vector3(end.point.x, end.point.y, end.point.z);
                    Debug.DrawLine(a, b, Color.black, 50f);

                    init.AddAdjacent(end, cost);
                    end.visitedInCreation = true;
                    before = end;

                    index++;

                }
            }
            static Point GeneratedPoint(Point init, Agent agent, Agent collision, bool negative = false)
            {
                Point vector1 = collision.position - init;
                Point vector2 = new Point(-vector1.z, 0, vector1.x);///Ortogonal
                float den = (float)Math.Sqrt(vector2.x * vector2.x + vector2.z * vector2.z);
                Point unitVector2 = vector2 / den;

                if (negative)
                    return collision.position + unitVector2 * (agent.radius + collision.radius) * -1.6f;
                return collision.position + unitVector2 * (agent.radius + collision.radius) * 1.6f;
            }
            static Point IntersectedOrtogonalVectors(Point init, Point end, Point obstacle)
            {
                Point vector1 = end - init;
                Point vectorInitToObstacle = obstacle - init;
                Point vector2 = new Point(-vectorInitToObstacle.z, 0, vectorInitToObstacle.x);///Ortogonal

                float
                    a = init.x, b = init.z, c = obstacle.x, d = obstacle.z,
                    x1 = vector1.x, x2 = vector2.x, y1 = vector1.z, y2 = vector2.z;

                float alfa = (a - (b * x1) / y1 - c + d * x1 / y1) / (x2 - x1 * y2 / y1);

                return obstacle + vector2 * alfa;
            }
        }
    }

}
