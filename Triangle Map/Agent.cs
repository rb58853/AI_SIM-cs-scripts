using System;
using System.Collections.Generic;
using BaseNode;
using DijkstraSpace;
using Point_Map;
using Triangle_Map;
using UnityEngine;

namespace Agent_Space
{
    public class Agent
    {
        public string name { get; private set; }
        public List<MapNode> ocupedNodes { get; private set; }
        public float radius { get; private set; }

        public MapNode currentNode { get; private set; }
        public Point position { get; private set; }
        public PointNode currentPosition { get; private set; }
        private PointNode nextPosition;

        public List<MapNode> triangleList { get; private set; }
        private PointPath pointPath;
        public List<PointNode> pointsMap { get; private set; }
        public Point destination { get; private set; }

        public Stack<Point> visualPath { get; private set; }
        public bool inMove { get; /*private*/ set; }

        /// <summary> Compatibility of this Agent whit a material.</summary>
        public Dictionary<Material, float> compatibility;

        public Agent(float radius, string name = "agent")
        {
            pointPath = new PointPath(this);
            this.name = name;
            compatibility = new Dictionary<Material, float>();
            visualPath = new Stack<Point>();
            ocupedNodes = new List<MapNode>();
            this.radius = radius;
            inMove = false;
        }


        public void SetCompatibility(Material material, float value)
        {
            if (compatibility.ContainsKey(material))
                compatibility[material] = value;
            else
                compatibility.Add(material, value);
        }

        public void searchCurrentNode()
        {
            foreach (Node node in Environment.map.nodes)
                if ((node as MapNode).triangle.PointIn(position))
                {
                    currentNode = node as MapNode;
                    ocupedNodes.Add(currentNode);
                    currentNode.AddAgent(this);
                    SetOcupedFromPosition(3);
                    break;
                }
        }
        public void setPosition(Point point) { position = point; }

        public void SetOcupedFromPosition(float extendArea = 1)
        {
            Queue<MapNode> ocuped = new Queue<MapNode>();

            ocuped.Enqueue(currentNode);

            if (!ocupedNodes.Contains(currentNode))
            {
                ocupedNodes.Add(currentNode);
                currentNode.AddAgent(this);
            }

            for (int i = 0; i < ocupedNodes.Count; i++)
            {
                MapNode node = ocupedNodes[i];
                if (node != currentNode)
                {
                    if (position.DistanceToTriangle(node.triangle) > radius * extendArea)
                    {
                        node.RemoveAgent(this);
                        ocupedNodes.Remove(node);
                        i -= 1;
                    }
                    else
                        ocuped.Enqueue(node);
                }
            }

            while (ocuped.Count > 0)
            {
                MapNode node = ocuped.Dequeue();

                foreach (MapNode adj in node.adjacents.Keys)
                    if (position.DistanceToTriangle(adj.triangle) < radius + extendArea)
                        if (!ocupedNodes.Contains(adj))
                        {
                            ocuped.Enqueue(adj);
                            ocupedNodes.Add(adj);
                            adj.AddAgent(this);
                        }
            }
        }

        Tuple<MapNode[], MapNode, MapNode> LocalMap(Point endPoint)
        {
            return BFS(endPoint);
        }
        Tuple<MapNode[], MapNode, MapNode> BFS(Point endPoint)
        {
            bool endWasFound = false;
            int countAfterFound = Environment.bfsArea;

            List<MapNode> localMap = new List<MapNode>();

            Dictionary<MapNode, MapNode> r = new Dictionary<MapNode, MapNode>();
            List<MapNode> visited = new List<MapNode>();//Mejorar eficiencia de esto, no me gusta

            Queue<MapNode> q = new Queue<MapNode>();

            MapNode end = null;

            r.Add(currentNode, new MapNode(currentNode, this, endPoint));
            visited.Add(currentNode);

            localMap.Add(r[currentNode]);


            if (currentNode.triangle.PointIn(endPoint))
                ///If endPoint is in Current node, return only current node as path, init and endNode
                return new Tuple<MapNode[], MapNode, MapNode>(localMap.ToArray(), r[currentNode], r[currentNode]);

            q.Enqueue(currentNode);

            while (q.Count > 0)
            {
                if (endWasFound == true)
                {
                    countAfterFound--;
                    if (countAfterFound <= 0) break;
                }

                MapNode n = q.Dequeue();
                foreach (MapNode adj in n.adjacents.Keys)
                {
                    Arist aristClone = n.adjacents[adj].Clone();

                    if (!visited.Contains(adj))
                    {
                        visited.Add(adj);
                        MapNode temp = new MapNode(adj, this, endPoint);

                        if (adj.triangle.PointIn(endPoint))
                        {
                            end = temp;
                            endWasFound = true;
                        }

                        r.Add(adj, temp);
                        r[n].AddAdjacent(temp, aristClone);
                        temp.AddAdjacent(r[n], aristClone);
                        aristClone.AddTriangle(r[n]);
                        aristClone.AddTriangle(temp);
                        //temp.SetDistance(Math.Min(temp.distance, r[n].distance + temp.Distance(n)));
                        q.Enqueue(adj);

                        localMap.Add(temp);
                    }
                    else
                    {
                        if (!r[n].adjacents.ContainsKey(r[adj]))
                        {
                            r[n].AddAdjacent(r[adj], aristClone);
                            r[adj].AddAdjacent(r[n], aristClone);
                            aristClone.AddTriangle(r[n]);
                            aristClone.AddTriangle(r[adj]);

                            //r[adj].SetDistance(Math.Min(r[adj].distance, r[n].distance + r[adj].Distance(n)));
                        }
                    }
                }
            }

            if (end == null)
                localMap = new List<MapNode>();

            return new Tuple<MapNode[], MapNode, MapNode>(localMap.ToArray(), r[currentNode], end);
        }

        public MapNode[] GetTrianglePath(Point endPoint, bool push = true)
        {
            Tuple<MapNode[], MapNode, MapNode> localMap = LocalMap(endPoint);
            Node[] nodes = localMap.Item1;
            Node init = localMap.Item2;
            Node end = localMap.Item3;

            if (end == null)
                return null;

            Dijkstra dijkstra = new Dijkstra(end, init, nodes);
            List<Node> path = dijkstra.GetPath();
            pointPath.PushCurrenTriangle((path[path.Count - 1] as MapNode).origin);

            DilatePath();
            triangleList = tools.ToListAsMapNode(path);

            MapNode[] result = tools.ToArrayAsMapNode(path);
            return result;

            void DilatePath()
            {
                int dilate = Environment.trianglePathDilatation;

                while (dilate > 0)
                {
                    dilate--;
                    MapNode temp = path[path.Count - 1] as MapNode;
                    Node[] array = path.ToArray();

                    path.Remove(temp);

                    foreach (MapNode node in array)
                        foreach (MapNode adj in node.adjacents.Keys)
                            if (!path.Contains(adj) && adj != temp)
                                path.Add(adj);

                    path.Add(temp);
                }

                foreach (MapNode node in path.ToArray())
                    if (node != path[0] && node != path[path.Count - 1])
                        if (OnlyOneAdj(node))
                            path.Remove(node);

                bool OnlyOneAdj(MapNode triangle)
                {
                    int countAdj = 0;
                    foreach (MapNode adj in triangle.adjacents.Keys)
                        if (path.Contains(adj))
                            countAdj++;

                    return countAdj <= 1;
                }
            }

        }

        public PointNode[] GetPointPath(Point endPoint)
        {
            MapNode[] tPath = GetTrianglePath(endPoint);

            if (tPath == null)
            {
                this.pointPath.PushPointMap(new PointNode[1] { new PointNode(position) });
                return new PointNode[1] { new PointNode(position) };///Debugguer
            }

            float density = Environment.densityPath;
            float mCost = currentNode.MaterialCost(this);

            PointNode endNode = new PointNode(endPoint, inArist: false);

            List<PointNode> mapPoints = PointNode.Static.CreatePointMap(endNode, position, this, density, mCost);

            ///MapPoints[0] = initNode
            ///MapPoints[MapPoints.Count-1] = endNode
            Dijkstra dijkstra = new Dijkstra(mapPoints[0], mapPoints[mapPoints.Count - 1], mapPoints.ToArray());
            List<Node> pointPath = dijkstra.GetPath(false);
            this.pointPath.PushPointMap(mapPoints);

            return tools.ToArrayAsPointNode(pointPath);///Debugguer
        }
        public void SetPointPath(Point point)
        {

            pointsMap = new List<PointNode>();
            destination = point;
            GetPointPath(point);
            NextPoint();
        }
        void DynamicSetPoint()
        {
            if (nextPosition.point.Distance(position) < 0.01f) return;

            float dist = radius * Environment.viewLenAgent;
            Point pointDest = position + Point.VectorUnit(position, nextPosition.point) * dist;

            Tuple<bool, Agent> collision = Collision(position, pointDest, this, ocupedNodes.ToArray(), 1.0f);
            if (collision.Item1)
            {
                NextPoint(true);
            }
        }

        int countMoves = 1;
        public void NextMove(int n = 1)
        {

            for (int i = 0; i < n; i++)
                NextMoveBasic();

            countMoves--;
            if (countMoves <= 0)
            {
                countMoves = 1;
                SetOcupedFromPosition(3);
            }
        }
        /// update frequence = (freq/[speed / 5]) frames.

        int freq = Environment.freqReview;
        void NextMoveBasic()
        {
            if (inMove)
            {
                if (visualPath.Count == 0) NextPoint();
                try { position = visualPath.Pop(); }
                catch { Debug.Log("Error: la pila tiene " + visualPath.Count + " elementos y esta intentando hacer Pop()."); }


                if (freq <= 0)
                {
                    freq = Environment.freqReview;
                    DynamicSetPoint();
                }
                freq--;
            }
        }
        void NextPoint(bool onCollision = false)
        {
            visualPath.Clear();

            if (!pointPath.empty)
            {
                inMove = true;

                nextPosition = pointPath.Pop(onCollision);
                currentPosition = pointPath.currentPoint;
                currentNode = pointPath.currentTriangle;

                //float cost = currentPosition.adjacents[nextPosition] * 25;
                float cost = currentNode.MaterialCost(this) * 25;

                List<Point> temp = new Arist(currentPosition.point, nextPosition.point).ToPoints(cost);
                for (int i = temp.Count - 1; i >= 0; i--)
                    visualPath.Push(temp[i]);
                NextMoveBasic();
            }
            else
                inMove = false;
        }

        public static Tuple<bool, Agent> Collision(Point node1, Point node2, Agent agent, MapNode mapNode, float multArea = 1)
        {
            Point l1 = node1;
            Point l2 = node2;
            float epsilon = 0.05f;
            Tuple<bool, Agent> result = new Tuple<bool, Agent>(false, null);

            foreach (Agent agentObstacle in mapNode.agentsIn)
            {
                if (agentObstacle == agent) continue;
                if (agentObstacle.position.DistanceToSegment(l1, l2) <= (agent.radius + agentObstacle.radius) * multArea + epsilon)
                    /// Collision
                    if (result.Item2 == null ||
                        agentObstacle.position.Distance(agent.position, false) <
                        result.Item2.position.Distance(agent.position, false))///mas cercano

                        result = new Tuple<bool, Agent>(true, agentObstacle);
            }
            if (result.Item1)
            {
                float distance = result.Item2.position.Distance(agent.position, false);
                float radius = result.Item2.radius + agent.radius;
                if (distance <= radius + epsilon)
                {
                    ///Choque
                    Point vector = Point.VectorUnit(result.Item2.position, agent.position) * (radius - distance + epsilon);
                    agent.position = agent.position + vector * 2f;
                    //collision.Item2.position = collision.Item2.position - vector;
                    //return new Tuple<bool, Agent>(false, null);
                }
            }

            return result;
        }
        public static Tuple<bool, Agent> Collision(Point node1, Point node2, Agent agent, MapNode[] mapNodes, float multArea = 1)
        {
            foreach (MapNode node in mapNodes)
            {
                Tuple<bool, Agent> collision = Collision(node1, node2, agent, node, multArea);
                if (collision.Item1)
                    return collision;
            }
            return new Tuple<bool, Agent>(false, null);
        }

        internal class tools
        {
            internal static MapNode[] ToArrayAsMapNode(List<Node> list)
            {
                MapNode[] result = new MapNode[list.Count];
                for (int i = 0; i < list.Count; i++)
                    result[i] = list[i] as MapNode;
                return result;
            }
            internal static List<MapNode> ToListAsMapNode(List<Node> list)
            {
                List<MapNode> result = new List<MapNode>();
                for (int i = 0; i < list.Count; i++)
                    result.Add(list[i] as MapNode);
                return result;
            }
            internal static PointNode[] ToArrayAsPointNode(List<Node> list)
            {
                PointNode[] result = new PointNode[list.Count];
                for (int i = 0; i < list.Count; i++)
                    result[i] = list[i] as PointNode;
                return result;
            }
        }
        public override string ToString()
        {
            return name;
        }
    }
}
