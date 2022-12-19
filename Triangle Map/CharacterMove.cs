using System;
using System.Collections.Generic;
using System.Text;
using Dijkstra_Algorithm;

namespace Triangle_Map
{
    public enum Material { basic = 10 }

    class Agent
    {
        public static Map map;
        private MapNode currentNode;

        /// <summary> Compatibility of this Agent whit a material.</summary>
        public Dictionary<Material, float> compatibility;

        public void SetCompatibility(Material material, float value)
        {
            if (compatibility.ContainsKey(material))
                compatibility[material] = value;
            else
                compatibility.Add(material, value);
        }

        public List<MapNode> getWayToNode(MapNode endNode)
        {
            Dijkstra dijkstra = new Dijkstra(map, currentNode, endNode, this);
            return dijkstra.GetWay();
        }
        public static void NewMap()
        {
            map = new Map();
        }
    }
    class Map
    {
        internal List<MapNode> nodes { get; private set; }
        public Map()
        {
            nodes = new List<MapNode>();
        }
        public void AddNode(MapNode node)
        {
            nodes.Add(node);
        }
    }
    class MapNode
    {
        internal Triangle triangle { get; private set; }
        public Dictionary<MapNode, Arist> adjacents { get; private set; }

        public Dictionary<Agent, bool> visited { get; private set; }
        public Dictionary<Agent, MapNode> father { get; private set; }
        
        /// <summary> Distance from the initial node of the way up to this node.</summary>
        public Dictionary<Agent, float> distance { get; private set; }
        /// <summary> The node of an agent's heap on which this node is located</summary>
        public Dictionary<Agent, HeapNode> heapNodes { get; private set; }

        public Material material { get; private set; }


        public MapNode(Triangle triangle)
        {
            this.triangle = triangle;
            adjacents = new Dictionary<MapNode, Arist>();
            DefaultValues();
        }
        public void SetMaterial(Material material)
        {
            this.material = material;
        }
        public void SetDistance(float value, Agent agent)
        {
            if (distance.ContainsKey(agent))
                distance[agent] = value;
            else
                distance.Add(agent,value);
        }
        public void SetFather(MapNode node, Agent agent)
        {
            if (father.ContainsKey(agent))
                father[agent] = node;
            else
                father.Add(agent, node);
        }
        public void SetVisited(Agent agent, bool value = true)
        {
            if (visited.ContainsKey(agent))
                visited[agent] = value;
            else
                visited.Add(agent, value);
        }
        public void SetHeapNode(HeapNode heapNode, Agent agent)
        {
            if (heapNodes.ContainsKey(agent))
                heapNodes[agent] = heapNode;
            else
                heapNodes.Add(agent, heapNode);
        }
        public void AddAdjacent(MapNode node, Arist arist)
        {
            adjacents.Add(node, arist);
        }
        public void AddAdjacent(MapNode node, Point p1, Point p2)
        {
            adjacents.Add(node, new Arist(p1, p2));
        }
        /// <summary>Total cost to move to a node with heuristics</summary>
        public float Value(Agent agent, MapNode end)
        {
            float g = distance[agent];
            float h = Heuristic(end);

            return g + h;
        }

        public float CostForMoveToNodeByAgent(MapNode node, Agent agent)
        {
            float material1 = (float)node.material / 10;
            float material2 = (float)this.material / 10;

            if (agent.compatibility.ContainsKey(node.material))
                material1 = agent.compatibility[node.material];
            if (agent.compatibility.ContainsKey(this.material))
                material2 = agent.compatibility[this.material];

            float averageMaterial = (material1 + material2) / 2;

            return node.EuclideanDistance(this) / averageMaterial;
        }

        float EuclideanDistance(MapNode node)
        {
            return triangle.EuclideanDistance(node.triangle);
        }
        float Heuristic(MapNode endNode)
        {
            return EuclideanDistance(endNode);
        }
        public void DefaultValues()
        {
            distance = new Dictionary<Agent, float>();
            heapNodes = new Dictionary<Agent, HeapNode>();
            father = new Dictionary<Agent, MapNode>();
            visited = new Dictionary<Agent, bool>();
        }
        public void DefaultValues(Agent agent)
        {
            if (distance.ContainsKey(agent))
                distance[agent] = float.MaxValue;
            else
                distance.Add(agent, float.MaxValue);
        }
        
        public int CompareTo(MapNode other, Agent agent, MapNode end)
        {
            return this.Value(agent,end).CompareTo(other.Value(agent,end));
        }
    }
    class Triangle
    {
        public Point vertex1 { get; private set; }
        public Point vertex2 { get; private set; }
        public Point vertex3 { get; private set; }
        public Point barycenter { get => Barycenter(); }


        public Triangle(Point v1, Point v2, Point v3)
        {
            this.vertex1 = v1;
            this.vertex2 = v2;
            this.vertex3 = v3;
        }
        public float EuclideanDistance(Triangle triangle)
        {
            return this.barycenter.Distance(triangle.barycenter);
        }
        Point Barycenter()
        {
            float x = (vertex1.x + vertex2.x + vertex3.x) / 3;
            float y = (vertex1.y + vertex2.y + vertex3.y) / 3;
            float z = (vertex1.z + vertex2.z + vertex3.z) / 3;
            return new Point(x, y, z);
        }

    }
    class Point
    {
        public float x { get; private set; }
        public float y { get; private set; }
        public float z { get; private set; }
        public Point(float x, float y, float z = 0)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public float Distance(Point point)
        {
            float temp = (point.x - this.x) * (point.x - this.x);
            temp += (point.y - this.y) * (point.y - this.y);
            temp += (point.z - this.z) * (point.z - this.z);
            return (float)Math.Sqrt(temp);
        }
    }
    class Arist
    {
        public Point p1 { get; private set; }
        public Point p2 { get; private set; }
        public Arist(Point p1, Point p2)
        {
            this.p1 = p1;
            this.p2 = p2;
        }
    }
    class WayPoints
    {
        public Point getProyectedWayPoint(Point point)
        {
            throw new NotImplementedException();
        }

    }
}


