using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using Agent_Space;
using BaseNode;

namespace Triangle_Map
{
    class MapNode : Node
    {
        public Agent_Space.Material material { get; private set; }

        internal Triangle triangle { get; private set; }
        public Dictionary<MapNode, Arist> adjacents { get; private set; }
        public MapNode origin { get; private set; }

        MapNode end;
        Agent agent;///esto importa por compatibilidad de materiales
        public MapNode(Triangle triangle)
        {
            this.triangle = triangle;
            adjacents = new Dictionary<MapNode, Arist>();

            this.agent = null;
            this.end = null;
            this.origin = null;

            DefaultValues();
        }
        public MapNode(MapNode origin, Agent agent, MapNode end = null)
        {
            this.triangle = origin.triangle;
            adjacents = new Dictionary<MapNode, Arist>();

            this.agent = agent;
            this.end = end;
            this.origin = origin;
            this.material = origin.material;
            DefaultValues();
        }
        public void SetEndNode(MapNode endNode) { this.end = endNode; }
        public void SetMaterial(Agent_Space.Material material) { this.material = material; }

        public void AddAdjacent(MapNode node, Arist arist) { adjacents.Add(node, arist); }
        public void AddAdjacent(MapNode node, Point p1, Point p2) { adjacents.Add(node, new Arist(p1, p2)); }
        public override float Value() { return distance + Heuristic(); }
        float EuclideanDistance(MapNode node)
        {
            return triangle.EuclideanDistance(node.triangle);
        }
        float Heuristic()
        {
            return EuclideanDistance(end);
        }
        public void DefaultValues()
        {
            distance = float.MaxValue;
            heapNode = null;
            father = null;
            visited = false;
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
            //float material1 = (float)node.material / 10;
            //float material2 = (float)this.material / 10;

            //if (agent.compatibility.ContainsKey(node.material))
            //    material1 = agent.compatibility[node.material];
            //if (agent.compatibility.ContainsKey(this.material))
            //    material2 = agent.compatibility[this.material];

            //float averageMaterial = (material1 + material2) / 2;

            return (node as MapNode).EuclideanDistance(this);
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
        public bool PointIn(Point p)
        {
            Point a = vertex1; Point b = vertex2; Point c = vertex3;
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
    class Arist
    {
        public Point p1 { get; private set; }
        public Point p2 { get; private set; }
        public Arist(Point p1, Point p2)
        {
            this.p1 = p1;
            this.p2 = p2;
        }
        public Arist(Arist a)
        {
            p1 = a.p1;
            p2 = a.p2;
        }
        public List<Point> ToPoints(float n = 1)
        {
            string eye;
            if (p1.x != p2.x)
                eye = "x";
            else
            {
                if (p1.y != p2.z)
                    eye = "y";
                else
                    eye = "z";
            }

            float x = (Point.Max(p1, p2, "x") - Point.Min(p1, p2, eye)).x;
            float y = (Point.Max(p1, p2, "x") - Point.Min(p1, p2, eye)).y;
            float z = (Point.Max(p1, p2, "x") - Point.Min(p1, p2, eye)).z;

            Point vector = new Point(x, y, z);

            List<Point> result = new List<Point>();
            result.Add(p1);
            result.Add(p2);

            float k = p1.Distance(p2) * n;

            for (int i = 1; i < k; i++)
            {
                float alfa = (float)i / (float)k;
                Point temp = Point.Min(p1, p2, eye) + vector * alfa;
                result.Add(temp);
            }
            return result;
        }

        public static List<Arist> ToAristList(MapNode[] path)
        {
            List<Arist> result = new List<Arist>();
            for (int i = 0; i < path.Length - 1; i++)
                result.Add(path[i].adjacents[path[i + 1]]);
            return result;
        }
    }
}