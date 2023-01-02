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
            return EuclideanDistance(node as PointNode) * adjacents[node as PointNode];
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
                float epsilon = 0.001f;
                foreach (Agent agentObstacle in mapNode.agentsIn)
                {
                    if (agentObstacle == agent) continue;
                    if (agentObstacle.position.DistanceToSegment(l1, l2) <= agent.radius + agentObstacle.radius + epsilon)
                        /// Collision
                        return new Tuple<bool, Agent>(true, agentObstacle);
                }
                return new Tuple<bool, Agent>(false, null);
            }
            public static Tuple<bool, Agent> Collision(Point node1, Point node2, Agent agent, MapNode[] mapNodes)
            {
                foreach (MapNode node in mapNodes)
                {
                    Tuple<bool, Agent> collision = Collision(node1, node2, agent, node);
                    if (collision.Item1)
                        return collision;
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
                            CreateObstacleBorder(current, end, agent, mapNode, cost, result, visitedObstacles, q, endList, endNode);
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

            static void CreateObstacleBorder(PointNode init, PointNode end,
                Agent agent, MapNode mapNode, float cost, List<PointNode> result,
                List<Agent> visitedObstacles, HeapNode q, List<PointNode> endList,
                PointNode endNode)
            {
                ///De alguna manera hay que hacer que esto pueda volver a visitar otros agentes para detectar colisiones 
                ///para poder hacer el bordeo mejor

                Tuple<bool, Agent> collision = Collision(init.point, end.point, agent, mapNode);
                if (collision.Item1)
                {
                    //DrawTwoPoints(init.point, collision.Item2.position, Color.black);

                    if (!visitedObstacles.Contains(collision.Item2))
                    {
                        visitedObstacles.Add(collision.Item2);

                        Point center = collision.Item2.position;

                        PointNode W = new PointNode(center -
                            Point.VectorUnit(init.point, center)
                            * (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder,
                            endNode);///West view from lefth to rigth

                        PointNode e = new PointNode(center +
                            Point.VectorUnit(init.point, center) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode);///Est view lefth to rigth

                        PointNode sw = new PointNode(W.point +
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode);///SouthWest view lefth to rigth

                        PointNode s = new PointNode(center +
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode);///South view lefth to rigth

                        PointNode se = new PointNode(e.point +
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode);///South view lefth to rigth

                        PointNode nw = new PointNode(W.point -
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode);///SouthWest view lefth to rigth

                        PointNode n = new PointNode(center -
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode);///South view lefth to rigth

                        PointNode ne = new PointNode(e.point -
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode);///South view lefth to rigth


                        //DrawTwoPoints(W.point, nw.point, Color.red);
                        //DrawTwoPoints(nw.point, n.point, Color.red);
                        //DrawTwoPoints(n.point, ne.point, Color.red);
                        //DrawTwoPoints(ne.point, ne.point, Color.red);



                        Queue<PointNode> up = new Queue<PointNode>();
                        /*up.Enqueue(W);*/
                        up.Enqueue(nw); /*up.Enqueue(n);*/ up.Enqueue(ne); up.Enqueue(e);
                        Queue<PointNode> down = new Queue<PointNode>();
                        down.Enqueue(sw); /*down.Enqueue(s);*/ down.Enqueue(se); down.Enqueue(e);


                        PointNode node1 = init;

                        while (up.Count > 0)
                        {
                            if (!node1.visitedInCreation) break;

                            PointNode node2 = up.Dequeue();

                            if (mapNode.triangle.PointIn(node2.point))
                                if (!Collision(node1.point, node2.point, agent, mapNode).Item1)
                                {
                                    node1.AddAdjacent(node2, cost);

                                    //DrawTwoPoints(node1.point, node2.point, Color.green);

                                    node2.visitedInCreation = true;
                                    node2.SetDistance(node1.distance + node1.EuclideanDistance(node2));
                                    q.Push(node2);
                                    result.Add(node2);
                                    endList.Add(node2);
                                }
                                else
                                {
                                    Debug.Log("encontro colision desde " + node1 + " hasta " + node2);
                                    CreateObstacleBorder(node1, node2, agent, mapNode, cost, result, visitedObstacles, q, endList, endNode);
                                }

                            node1 = node2;
                        }

                        node1 = W;
                        while (down.Count > 0)
                        {
                            if (!node1.visitedInCreation) break;

                            PointNode node2 = down.Dequeue();
                            if (mapNode.triangle.PointIn(node2.point))
                                if (!Collision(node1.point, node2.point, agent, mapNode).Item1)
                                {
                                    node1.AddAdjacent(node2, cost);

                                    //DrawTwoPoints(node1.point, node2.point, Color.green);

                                    node2.visitedInCreation = true;
                                    node2.SetDistance(node1.distance + node1.EuclideanDistance(node2));
                                    q.Push(node2);
                                    result.Add(node2);
                                    endList.Add(node2);
                                }
                                else
                                {
                                    Debug.Log("encontro colision desde " + node1 + " hasta " + node2);
                                    CreateObstacleBorder(node1, node2, agent, mapNode, cost, result, visitedObstacles, q, endList, endNode);
                                }

                            node1 = node2;
                        }
                    }
                }
            }
            static Point GeneratedPoint(Point init, Agent agent, Agent collision, bool negative = false)
            {
                Point vector1 = collision.position - init;
                Point vector2 = new Point(-vector1.z, 0, vector1.x);///Ortogonal
                float den = (float)Math.Sqrt(vector2.x * vector2.x + vector2.z * vector2.z);
                Point unitVector2 = vector2 / den;

                float n = Agent_Space.Environment.collisionBorder;
                if (negative)
                    return collision.position + unitVector2 * (agent.radius + collision.radius) * -n;
                return collision.position + unitVector2 * (agent.radius + collision.radius) * n;
            }
            static void DrawTwoPoints(Point p1, Point p2, Color color)
            {
                Vector3 a = new Vector3(p1.x, p1.y, p1.z);
                Vector3 b = new Vector3(p2.x, p2.y, p2.z);
                Debug.DrawLine(a, b, color, 50f);
            }
        }
    }

}
