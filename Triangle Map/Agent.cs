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

        private MapNode currentNode;
        private Queue<MapNode> trianglePath;

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










        public void setCurrentNode(MapNode node) { currentNode = node; }


        Tuple<MapNode[], MapNode, MapNode> LocalMap(MapNode endNode)
        {
            return BFS(endNode);
        }
        Tuple<MapNode[], MapNode, MapNode> BFS(MapNode endNode)
        {
            List<MapNode> localMap = new List<MapNode>();

            Dictionary<MapNode, MapNode> r = new Dictionary<MapNode, MapNode>();
            List<MapNode> visited = new List<MapNode>();//Mejorar eficiencia de esto, no me gusta

            Queue<MapNode> q = new Queue<MapNode>();

            MapNode end = new MapNode(endNode, this, endNode); end.SetEndNode(end);

            r.Add(currentNode, new MapNode(currentNode, this, end));
            visited.Add(currentNode);

            localMap.Add(r[currentNode]);
            r[currentNode].SetDistance(0);

            q.Enqueue(currentNode);

            while (q.Count > 0)
            {
                MapNode n = q.Dequeue();
                foreach (MapNode adj in n.adjacents.Keys)
                {
                    if (!visited.Contains(adj))
                    {
                        visited.Add(adj);
                        MapNode temp = new MapNode(adj, this, end);

                        if (adj == endNode)
                        {
                            temp = end;
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

            return new Tuple<MapNode[], MapNode, MapNode>(localMap.ToArray(), r[currentNode], end);
        }

        public List<Node> GetTrianglePath(MapNode endNode)
        {
            Tuple<MapNode[], MapNode, MapNode> localMap = LocalMap(endNode);
            Node[] nodes = localMap.Item1;
            Node init = localMap.Item2;
            Node end = localMap.Item3;

            Dijkstra dijkstra = new Dijkstra(init, end, nodes);
            return dijkstra.GetPath();
        }
        public List<Arist> GetAritsPath(MapNode endNode)
        {
            Tuple<MapNode[], MapNode, MapNode> localMap = LocalMap(endNode);
            Node[] nodes = localMap.Item1; Node init = localMap.Item2; Node end = localMap.Item3;
            Dijkstra dijkstra = new Dijkstra(init, end, nodes);
            MapNode[] path = tools.ToArrayAsMapNode(dijkstra.GetPath());/// Si se puede, mejorar la eficiencia con lo default

            return Arist.ToAristList(path);
        }
        public PointNode[] GetPointPath(MapNode end)
        {
            //= map.nodes[170] as MapNode;
            Point p1 = currentNode.triangle.barycenter;
            Point p2 = end.triangle.barycenter;
            List<Arist> aritPath = GetAritsPath(end);
            List<PointNode> mapPoints = PointNode.Static.CreatePointMap(aritPath, p1, p2);

            if (mapPoints.Count == 0)
                return new PointNode[0];

            ///MapPoints[0] = endNode
            ///MapPoints[1] = initNode
            Dijkstra dijkstra = new Dijkstra(mapPoints[1], mapPoints[0], mapPoints.ToArray());
            List<Node> pointPath = dijkstra.GetPath();

            return tools.ToArrayAsPointNode(pointPath);
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
