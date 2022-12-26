using System;
using System.Collections.Generic;
using BaseNode;
using DijkstraSpace;
using Point_Map;
using Triangle_Map;
using UnityEngine;

namespace Agent_Space
{
    public enum Material { basic = 10 }

    class Agent
    {
        public static Map map;

        public MapNode currentNode { get; private set; }
        private Point position;
        private Queue<MapNode> trianglePath;
        private Stack<PointNode> pointPath;

        /// <summary> Compatibility of this Agent whit a material.</summary>
        public Dictionary<Material, float> compatibility;

        public Agent()
        {
            compatibility = new Dictionary<Material, float>();
        }
        public void SetCompatibility(Material material, float value)
        {
            if (compatibility.ContainsKey(material))
                compatibility[material] = value;
            else
                compatibility.Add(material, value);
        }
        public static void NewMap()
        {
            map = new Map();
        }








        public void searchCurrentNode()
        {
            foreach (Node node in map.nodes)
                if ((node as MapNode).triangle.PointIn(position))
                {
                    currentNode = node as MapNode;
                    break;
                }
        }

        void setCurrentNode(MapNode node) { currentNode = node; }
        public void setPosition(Point point) { position = point; }


        Tuple<MapNode[], MapNode, MapNode> LocalMap(Point endPoint)
        {
            return BFS(endPoint);
        }
        Tuple<MapNode[], MapNode, MapNode> BFS(Point endPoint)
        {

            List<MapNode> localMap = new List<MapNode>();

            Dictionary<MapNode, MapNode> r = new Dictionary<MapNode, MapNode>();
            List<MapNode> visited = new List<MapNode>();//Mejorar eficiencia de esto, no me gusta

            Queue<MapNode> q = new Queue<MapNode>();

            //MapNode end = new MapNode(endNode, this, endNode); end.SetEndNode(end);
            MapNode end = null;

            //r.Add(currentNode, new MapNode(currentNode, this, end));
            r.Add(currentNode, new MapNode(currentNode, this));
            visited.Add(currentNode);

            localMap.Add(r[currentNode]);
            //r[currentNode].SetDistance(0);


            if (currentNode.triangle.PointIn(endPoint))
            {
                r[currentNode].SetEndNode(r[currentNode]);
                return new Tuple<MapNode[], MapNode, MapNode>(localMap.ToArray(), r[currentNode], r[currentNode]);
            }

            q.Enqueue(currentNode);

            while (q.Count > 0)
            {
                MapNode n = q.Dequeue();
                foreach (MapNode adj in n.adjacents.Keys)
                {
                    if (!visited.Contains(adj))
                    {
                        visited.Add(adj);
                        MapNode temp = new MapNode(adj, this);

                        if (adj.triangle.PointIn(endPoint))
                        {
                            end = temp;
                            //Hacer mas cosas
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

            if (end != null)
                foreach (MapNode node in localMap)
                    node.SetEndNode(end);
            else
                localMap = new List<MapNode>();

            return new Tuple<MapNode[], MapNode, MapNode>(localMap.ToArray(), r[currentNode], end);
        }

        public Node[] GetTrianglePath(Point endPoint)
        {
            Tuple<MapNode[], MapNode, MapNode> localMap = LocalMap(endPoint);
            Node[] nodes = localMap.Item1;
            Node init = localMap.Item2;
            Node end = localMap.Item3;

            Dijkstra dijkstra = new Dijkstra(init, end, nodes);
            return tools.ToArrayAsMapNode(dijkstra.GetPath());
        }
        List<Arist> GetAritsPath(Point endPoint)
        {
            Tuple<MapNode[], MapNode, MapNode> localMap = LocalMap(endPoint);
            Node[] nodes = localMap.Item1; Node init = localMap.Item2; Node end = localMap.Item3;
            Dijkstra dijkstra = new Dijkstra(init, end, nodes);

            if (nodes.Length == 0)
                throw new Exception("No existe camino");

            MapNode[] path = tools.ToArrayAsMapNode(dijkstra.GetPath());/// Si se puede, mejorar la eficiencia con lo default

            return Arist.ToAristList(path);
        }
        public PointNode[] GetPointPath(Point endPoint)
        {
            float n = 12f;///Density
            List<Arist> aritPath = GetAritsPath(endPoint);
            List<PointNode> mapPoints = PointNode.Static.CreatePointMap(aritPath, position, endPoint, n);

            if (mapPoints.Count == 0)
                return new PointNode[1] { new PointNode(position) };

            ///MapPoints[0] = endNode
            ///MapPoints[1] = initNode
            Dijkstra dijkstra = new Dijkstra(mapPoints[1], mapPoints[0], mapPoints.ToArray());
            List<Node> pointPath = dijkstra.GetPath();

            return tools.ToArrayAsPointNode(pointPath);
        }
        public void SetPointPath(Point point)
        {
            PointNode[] path = GetPointPath(point);
            for (int i = path.Length - 1; i >= 0; i--)
                pointPath.Push(path[i]);
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
    }

}
