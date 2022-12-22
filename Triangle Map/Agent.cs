using System.Collections.Generic;
using BaseNode;
using DijkstraSpace;
using Point_Map;
using Triangle_Map;

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

        List<MapNode> localMap;
        MapNode[] LocalMap(MapNode endNode)
        {
            //esto es hacerlo con BFS y cosa
            localMap = new List<MapNode>();
            MapNode node = new MapNode(origin: currentNode, end: endNode, agent: this);
            BFS(node,endNode);

            return localMap.ToArray();
        }
        void BFS(MapNode node, MapNode endNode)
        {
            /// Esta es la pincha actual, iterar por el bfs
        }

        public List<Node> GetTrianglePath(MapNode endNode)
        {
            Node[] nodes = LocalMap(endNode);
            Dijkstra dijkstra = new Dijkstra(currentNode, endNode, nodes);
            return dijkstra.GetPath();
        }
        public List<Arist> GetAritsPath(MapNode endNode)
        {
            Dijkstra dijkstra = new Dijkstra(currentNode, endNode, map.nodes.ToArray());
            return Arist.ToAristList(dijkstra.GetPath().ToArray() as MapNode[]);
        }
       
        public PointNode[] GetPointPath(MapNode end)
        {
             //= map.nodes[170] as MapNode;
            Point p1 = currentNode.triangle.barycenter;
            Point p2 = end.triangle.barycenter;
            List<Arist> aritPath = GetAritsPath(end);
            List<PointNode> mapPoints = PointNode.CreatePointMap(aritPath, p1, p2);

            if (mapPoints.Count == 0)
                return new PointNode[0];

            Node[] test = mapPoints.ToArray();

            Dijkstra dijkstra = new Dijkstra(mapPoints[0], mapPoints[mapPoints.Count - 1], mapPoints.ToArray());
            List<Node> pointPath = dijkstra.GetPath();
            return pointPath.ToArray() as PointNode[];
        }
    }
}