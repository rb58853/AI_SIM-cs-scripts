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

        public Queue<MapNode> trianglePath { get; private set; }
        private Stack<PointNode> pointPath;
        public Point destination { get; private set; }

        public Stack<Point> visualPath { get; private set; }
        public bool inMove { get; private set; }

        /// <summary> Compatibility of this Agent whit a material.</summary>
        public Dictionary<Material, float> compatibility;

        public Agent(float radius, string name = "agent")
        {
            this.name = name;
            compatibility = new Dictionary<Material, float>();
            trianglePath = new Queue<MapNode>();
            pointPath = new Stack<PointNode>();
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

                    SetOcupedFromPosition();
                    break;
                }
        }
        public void setPosition(Point point) { position = point; }

        public void SetOcupedFromPosition()
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
                    if (position.DistanceToTriangle(node.triangle) > radius)
                    {
                        node.RemoveAgent(this);
                        ocupedNodes.Remove(node);
                        i -= 1;
                    }
                    else
                        ocuped.Enqueue(node);
                }
            }

            //int o = 0;
            while (ocuped.Count > 0)
            {
                MapNode node = ocuped.Dequeue();

                foreach (MapNode adj in node.adjacents.Keys)
                    if (position.DistanceToTriangle(adj.triangle) < radius)
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
                        r[n].AddAdjacent(temp, n.adjacents[adj]);
                        temp.AddAdjacent(r[n], n.adjacents[adj]);
                        //temp.SetDistance(Math.Min(temp.distance, r[n].distance + temp.Distance(n)));
                        q.Enqueue(adj);

                        localMap.Add(temp);
                    }
                    else
                    {
                        if (!r[n].adjacents.ContainsKey(r[adj]))
                        {
                            r[n].AddAdjacent(r[adj], n.adjacents[adj]);
                            r[adj].AddAdjacent(r[n], n.adjacents[adj]);
                            //r[adj].SetDistance(Math.Min(r[adj].distance, r[n].distance + r[adj].Distance(n)));
                        }
                    }
                }
            }

            if (end == null)
                localMap = new List<MapNode>();

            return new Tuple<MapNode[], MapNode, MapNode>(localMap.ToArray(), r[currentNode], end);
        }

        public MapNode[] GetTrianglePath(Point endPoint)
        {
            trianglePath.Clear();

            Tuple<MapNode[], MapNode, MapNode> localMap = LocalMap(endPoint);
            Node[] nodes = localMap.Item1;
            Node init = localMap.Item2;
            Node end = localMap.Item3;

            if (end == null)
                return null;

            Dijkstra dijkstra = new Dijkstra(init, end, nodes);

            MapNode[] result = tools.ToArrayAsMapNode(dijkstra.GetPath());

            foreach (MapNode node in result)
                trianglePath.Enqueue(node);

            return result;
        }
        List<Arist> GetAritsPath(Point endPoint)
        {
            //Tuple<MapNode[], MapNode, MapNode> localMap = LocalMap(endPoint);
            //Node[] nodes = localMap.Item1; Node init = localMap.Item2; Node end = localMap.Item3;
            //Dijkstra dijkstra = new Dijkstra(init, end, nodes);

            //if (nodes.Length == 0)
            //    return null;

            MapNode[] path = GetTrianglePath(endPoint);
            if (path == null) return null;
            return Arist.ToAristList(path);
        }
        public PointNode[] GetPointPath(Point endPoint)
        {
            List<Arist> aritPath = GetAritsPath(endPoint);

            if (aritPath == null)
                return new PointNode[1] { new PointNode(position) };

            float density = Environment.densityPath;
            float mCost = currentNode.MaterialCost(this);
            List<PointNode> mapPoints = PointNode.Static.CreatePointMap(aritPath, position, endPoint, this, density, mCost);

            ///MapPoints[0] = initNode
            ///MapPoints[MapPoints.Count-1] = endNode
            Dijkstra dijkstra = new Dijkstra(mapPoints[0], mapPoints[mapPoints.Count - 1], mapPoints.ToArray());
            List<Node> pointPath = dijkstra.GetPath();

            return tools.ToArrayAsPointNode(pointPath);
        }
        public void SetPointPath(Point point)
        {
            destination = point;
            pointPath.Clear();
            //GetTrianglePath(point);
            PointNode[] path = GetPointPath(point);
            for (int i = path.Length - 1; i >= 0; i--)
                pointPath.Push(path[i]);

            nextPosition = pointPath.Pop();
            currentPosition = null;
            NextPoint();
        }

        int countMoves = 5;
        public void NextMove(int n = 1)
        {

            for (int i = 0; i < n; i++)
                NextMoveBasic();

            countMoves--;
            if (countMoves <= 0)
            {
                //if (PointNode.Static.Collision(position, pointPath.ToArray()[0].point, this, currentNode).Item1)
                //SetPointPath(destination);
                ///Se le puede decir tambien que cambie la direccion y mantenga el camino ya calculado, habria que disennar un algoritmo que 
                ///funcione con ello, pero no habra nada que lo haga preciso

                countMoves = 1;
                SetOcupedFromPosition();
                //SetPointPath(destination);
            }
        }
        void NextMoveBasic()
        {
            if (inMove)
            {
                if (visualPath.Count == 0) NextPoint();
                try { position = visualPath.Pop(); }
                catch { Debug.Log("Error: la pila tiene " + visualPath.Count + " elementos y esta intentando hacer Pop()."); }
            }
        }
        void NextPoint()
        {
            if (pointPath.Count == 0)
            {
                inMove = false;
                return;
            }
            else
                inMove = true;

            visualPath.Clear();


            try { currentNode = trianglePath.Dequeue().origin; }
            catch { Debug.Log("La cola tiene " + trianglePath.Count + " elementos y esta intentando hacer Dequeue()"); }

            currentPosition = nextPosition;
            nextPosition = pointPath.Pop();

            float cost = currentPosition.adjacents[nextPosition] * 25;

            List<Point> temp = new Arist(currentPosition.point, nextPosition.point).ToPoints(cost);

            for (int i = temp.Count - 1; i >= 0; i--)
                visualPath.Push(temp[i]);

            NextMoveBasic();
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
