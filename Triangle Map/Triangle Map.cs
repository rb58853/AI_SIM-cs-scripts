using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using Agent_Space;
using BaseNode;
using Point_Map;

namespace Triangle_Map
{
    public class MapNode : Node
    {
        public Agent_Space.Material material { get; private set; }

        internal Triangle triangle { get; private set; }
        public Dictionary<MapNode, Arist> adjacents { get; private set; }
        public MapNode origin { get; private set; }
        public float materialCost { get => MaterialCost(); }
        public List<Agent> agentsIn { get; private set; }

        Point end;
        Agent agent;///esto importa por compatibilidad de materiales
        public MapNode(Triangle triangle)
        {
            this.triangle = triangle;
            adjacents = new Dictionary<MapNode, Arist>();
            agentsIn = new List<Agent>();
            this.agent = null;
            this.end = null;
            this.origin = null;

            DefaultValues();
        }
        public MapNode(MapNode origin, Agent agent, Point end = null)
        {
            this.triangle = origin.triangle;
            adjacents = new Dictionary<MapNode, Arist>();
            agentsIn = new List<Agent>();

            this.agent = agent;
            this.end = end;
            this.origin = origin;
            DefaultValues();
            this.material = origin.material;
        }
        public void AddAgent(Agent agent)
        {
            agentsIn.Add(agent);
        }
        public void RemoveAgent(Agent agent)
        {
            agentsIn.Remove(agent);
        }
        public void SetEndPoint(Point point) { this.end = point; }
        public void SetMaterial(Agent_Space.Material material) { this.material = material; }

        public void AddAdjacent(MapNode node, Arist arist) { adjacents.Add(node, arist); }
        public void AddAdjacent(MapNode node, Point p1, Point p2) { adjacents.Add(node, new Arist(p1, p2)); }
        public override float Value()
        {
            float hWeigth = Agent_Space.Environment.heuristicTriangleWeigth;
            float gWeigth = 1;
            return distance * gWeigth + Heuristic() * hWeigth;
        }
        float EuclideanDistance(MapNode node)
        {
            return triangle.EuclideanDistance(node.triangle);
        }
        float Heuristic()
        {
            return triangle.barycenter.Distance(end);
        }
        public void DefaultValues()
        {
            distance = float.MaxValue;
            heapNode = null;
            father = null;
            visited = false;
            SetMaterial(Agent_Space.Material.basic);
        }
        public override List<Node> GetAdyacents()
        {
            ///Esto hay que mejorarlo en en futuro, itera por todo
            List<Node> result = new List<Node>();
            foreach (Node adj in adjacents.Keys)
                result.Add(adj);
            return result;
        }
        public override string ToString()
        {
            return triangle.ToString();
        }

        public override float Distance(Node node)
        {
            float density = Agent_Space.Environment.densityPath;
            List<Point> points = adjacents[node as MapNode].ToPoints(density,agent.pointsMap);
            Point mid = MinMid(points, triangle.barycenter, (node as MapNode).triangle.barycenter);

            //drawToNode(node, mid);

            float d1 = triangle.barycenter.Distance(mid) * MaterialCost();
            float d2 = (node as MapNode).triangle.barycenter.Distance(mid) * (node as MapNode).MaterialCost();

            return d1 + d2;
        }
        Point MinMid(List<Point> mids, Point p1, Point p2)
        {
            float minDist = float.MaxValue;
            Point result = mids[0];
            foreach (Point point in mids)
            {
                float tempDist = p1.Distance(point) + p2.Distance(point);
                if (tempDist < minDist)
                {
                    result = point;
                    minDist = tempDist;
                }
            }
            return result;
        }

        void drawToNode(Node node, Point mid)
        {
            Vector3 p1 = new Vector3(triangle.barycenter.x, triangle.barycenter.y, triangle.barycenter.z);
            Vector3 p2 = new Vector3(mid.x, mid.y, mid.z);

            Debug.DrawLine(p1, p2, Color.black, 50f);
            p1 = new Vector3((node as MapNode).triangle.barycenter.x, (node as MapNode).triangle.barycenter.y, (node as MapNode).triangle.barycenter.z);
            Debug.DrawLine(p1, p2, Color.black, 50f);
        }
        float MaterialCost()
        {
            if (agent.compatibility.ContainsKey(this.material))
                return agent.compatibility[this.material];
            return (float)this.material / 10f;
        }
        public float MaterialCost(Agent agent)
        {
            if (agent.compatibility.ContainsKey(this.material))
                return agent.compatibility[this.material];
            return (float)this.material / 10f;
        }
    }
    public class Triangle
    {
        public Point vertex1 { get; private set; }
        public Point vertex2 { get; private set; }
        public Point vertex3 { get; private set; }
        public Point barycenter { get => Barycenter(); }
        public float maxSide { get => Math.Max(Math.Max(vertex1.Distance(vertex2), vertex1.Distance(vertex3)), vertex2.Distance(vertex3)); }

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
        public bool PointIn(Point p)
        {
            Point a = vertex1; Point b = vertex2; Point c = vertex3;
            if (c.z - a.z == 0)
            {
                a = vertex2;
                b = vertex1;
            }

            Point d = b - a; Point e = c - a;

            float w1 = (e.x * (a.z - p.z) + e.z * (p.x - a.x)) / (d.x * e.z - d.z * e.x);
            float w2 = (p.z - a.z - w1 * d.z) / e.z;

            return (w1 >= 0.0) && (w2 >= 0.0) && ((w1 + w2) <= 1.0);
        }
        public override string ToString()
        {
            return barycenter.ToString();
        }
    }
    public class Arist
    {
        public float materialCost { get; private set; }
        public Point p1 { get; private set; }
        public Point p2 { get; private set; }
        public List<PointNode> points { get; private set; }
        public Arist(Point p1, Point p2)
        {
            this.p1 = p1;
            this.p2 = p2;
            points = new List<PointNode>();
        }
        public Arist(Arist a)
        {
            p1 = a.p1;
            p2 = a.p2;
            points = new List<PointNode>();
        }
        public List<Point> ToPoints(float n = 1f, List<PointNode> map = null, bool set = false)
        {
            Point vector = p2 - p1;

            List<Point> result = new List<Point>();
            points = new List<PointNode>();

            float k = p1.Distance(p2) * n;

            result.Add(p1);

            PointNode node = new PointNode(p1);
            points.Add(node);

            if (map != null)
                map.Add(node);

            for (int i = 1; i < k; i++)
            {
                float alfa = (float)i / (float)k;
                result.Add(p1 + vector * alfa);
                node = new PointNode(p1 + vector * alfa);
                points.Add(node);

                if (map != null)
                    map.Add(node);
            }
            result.Add(p2);
            node = new PointNode(p2);
            points.Add(node);

            if (map != null)
                map.Add(node);

            return result;
        }
        public void addPointsToMap(List<PointNode> map)
        {
            foreach (PointNode point in points)
                map.Add(point);
        }
        public void SetMaterialCost(float value)
        {
            materialCost = value;
        }
        public static List<Arist> ToAristList(MapNode[] path)
        {
            List<Arist> result = new List<Arist>();
            if (path.Length == 0) return result;

            for (int i = 0; i < path.Length - 1; i++)
            {
                Arist temp = path[i].adjacents[path[i + 1]];
                temp.SetMaterialCost(path[i + 1].materialCost);
                result.Add(temp);
            }


            return result;
        }
        public Arist Clone()
        {
            return new Arist(this);
        }
    }
}