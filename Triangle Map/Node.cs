﻿using System;
using System.Collections.Generic;
using DijkstraSpace;
using Point_Map;
using Triangle_Map;
using UnityEngine;
namespace BaseNode
{
    public abstract class Node : IComparable<Node>
    {
        public float value { get => Value(); }
        public bool visited { get; protected set; }
        public Node father { get; protected set; }
        public float distance { get; protected set; }

        /// <summary> This HeapNode is used by Heapify in Relax .</summary>
        public Heap heapNode { get; protected set; }

        public abstract float Value();
        public abstract List<Node> GetAdyacents();
        public abstract float Distance(Node node);

        public virtual void SetDistance(float value) { distance = value; }
        public virtual void SetFather(Node node) { father = node; }
        public virtual void SetVisited(bool value = true) { visited = value; }
        public virtual void SetHeapNode(Heap heapNode) { this.heapNode = heapNode; }
        public int CompareTo(Node other) { return this.value.CompareTo(other.value); }
    }
    class Map
    {
        internal List<Node> nodes { get; private set; }

        public Map()
        {
            nodes = new List<Node>();
        }
        public Map(List<PointNode> nodes)
        {
            this.nodes = new List<Node>();
        }
        public void AddNode(Node node)
        {
            nodes.Add(node);
        }
    }
    public class Point
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
        public float Distance(Point point, bool D3 = true)
        {

            float temp = (point.x - this.x) * (point.x - this.x);

            if (D3) temp += (point.y - this.y) * (point.y - this.y);

            temp += (point.z - this.z) * (point.z - this.z);
            return (float)Math.Sqrt(temp);
        }
        public static Point operator *(Point point, float a)
        {
            return new Point(a * point.x, a * point.y, point.z * a);
        }
        public static Point operator /(Point point, float a)
        {
            return new Point(point.x / a, point.y / a, point.z / a);
        }

        public static Point operator +(Point point1, Point point2)
        {
            return new Point(point1.x + point2.x, point1.y + point2.y, point1.z + point2.z);
        }
        public static Point operator -(Point point1, Point point2)
        {
            return new Point(point1.x - point2.x, point1.y - point2.y, point1.z - point2.z);
        }
        public static Point Max(Point point1, Point point2, string eye)
        {
            if (eye == "x")
            {
                if (point1.x > point2.x)
                    return point1;
                return point2;
            }
            if (eye == "y")
            {
                if (point1.y > point2.y)
                    return point1;
                return point2;
            }
            if (point1.z > point2.z)
                return point1;
            return point2;

        }
        public static Point Min(Point point1, Point point2, string eye)
        {
            if (eye == "x")
            {
                if (point1.x < point2.x)
                    return point1;
                return point2;
            }
            if (eye == "y")
            {
                if (point1.y < point2.y)
                    return point1;
                return point2;
            }
            if (point1.z < point2.z)
                return point1;
            return point2;

        }
        public override string ToString()
        {
            return "<" + x + "; " + y + "; " + z + ">";
        }
        public Vector3 ToVector3()
        {
            return new Vector3(x, y, z);
        }
        public float DistanceToLine(Point l1, Point l2)
        {
            Point point = OrtogonalProyection(l1, l2, this);
            return Distance(point, false);
        }
        public float DistanceToSegment(Point l1, Point l2)
        {
            l1 = new Point(l1.x, 0, l1.z);
            l2 = new Point(l2.x, 0, l2.z);

            float lenSegment = l1.Distance(l2, false);
            Point intersected = OrtogonalProyection(l1, l2, this);
            bool inSegment = intersected.Distance(l1, false) <= lenSegment && intersected.Distance(l2, false) <= lenSegment;

            if (inSegment)
                return this.Distance(intersected, false);
            else
                return Math.Min(this.Distance(l1, false), this.Distance(l2, false));
        }
        public float DistanceToTriangle(Triangle triangle)
        {
            float d1 = DistanceToSegment(triangle.vertex1, triangle.vertex2);
            float d2 = DistanceToSegment(triangle.vertex1, triangle.vertex3);
            float d3 = DistanceToSegment(triangle.vertex3, triangle.vertex2);
            return Math.Min(Math.Min(d1, d2), d3);
        }
        public static Point OrtogonalProyection(Point l1, Point l2, Point point)
        {
            Point vector1 = l2 - l1;
            Point vector2 = OrtogonalVector(vector1);/// Ortogonal from point to vector 

            ///Vectors intersections
            if (vector1.x == 0)
                return new Point(l1.x, 0, point.z);
            if (vector1.z == 0)
                return new Point(point.x, 0, l1.z);

            float
                a = l1.x, b = l1.z, c = point.x, d = point.z,
                x1 = vector1.x, x2 = vector2.x, y1 = vector1.z, y2 = vector2.z;

            float alfa = (a - (b * x1) / y1 - c + d * x1 / y1) / (x2 - x1 * y2 / y1);

            return point + vector2 * alfa;
        }
        public static Point OrtogonalVector(Point p1, Point p2)
        {
            Point vector = p2 - p1;
            return new Point(-vector.z, 0, vector.x);///Ortogonal
        }
        public static Point OrtogonalVector(Point vector)
        {
            return new Point(-vector.z, 0, vector.x);///Ortogonal
        }
        public static Point VectorUnit(Point init, Point end, bool D3 = false)
        {
            Point temp = (end - init);
            if (!D3) temp = new Point(temp.x, 0, temp.z);
            return temp / (float)Math.Sqrt((temp.x * temp.x + temp.z * temp.z));
        }
        public static Point VectorUnit(Point vector)
        {
            return vector / (float)Math.Sqrt((vector.x * vector.x + vector.z * vector.z));
        }

    }
}