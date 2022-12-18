using System;
using System.Collections.Generic;

namespace DijkstraSpace
{
    class Dijkstra
    {
        public static float g_weigth { get; private set; }
        public static float h_weigth { get; private set; }
        public Dijkstra(float g_weigth, float h_weigth)
        {
            Dijkstra.g_weigth = g_weigth;
            Dijkstra.h_weigth = h_weigth;
        }

        TimeSpan Time;
        public DijkstraNode END { get; protected set; }
        public void GridToDijktraGraph(DijkstraNode[,] grid)
        {
            Random random = new Random();
            for (int y = 0; y < grid.GetLength(0); y++)
                for (int x = 0; x < grid.GetLength(1); x++)
                    for (int i = -1; i <= 1; i++)
                        for (int j = -1; j <= 1; j++)
                            if ((i != 0 || j != 0) && x + i >= 0 && y + j >= 0 && y + j <= grid.GetLength(0) - 1 && x + i <= grid.GetLength(1) - 1)
                            {
                                grid[x, y].AddAdj(grid[x + i, y + j], random.Next(1, 3));
                                grid[x, y].SetPosition(new Point(x, y));
                            }
        }

        public virtual void Start(List<DijkstraNode> graph, DijkstraNode init, DijkstraNode end)
        {
            init.SetDist(0);
            END = end;

            DijkstraNode root = graph[0];
            HeapNode heap = new HeapNode(root);
            foreach (DijkstraNode node in graph.GetRange(1, graph.Count - 1))
                heap.Push(node);

            Calculate(heap, end);

            Console.WriteLine("Caso Distrack:");
            Print(graph);
        }
        public virtual void Start(DijkstraNode[,] grid, DijkstraNode init, DijkstraNode end)
        {
            List<DijkstraNode> graph = new List<DijkstraNode>();
            for (int i = 0; i < grid.GetLength(0); i++)
                for (int j = 0; j < grid.GetLength(1); j++)
                    graph.Add(grid[i, j]);
            Start(graph, init, end);
        }

        public void Calculate(HeapNode heap, DijkstraNode end)
        {
            DateTime t0 = DateTime.Now;

            Stack<DijkstraNode> A = new Stack<DijkstraNode>();

            HeapNode Q = heap;

            while (Q.size > 0)
            {
                DijkstraNode node = Q.Pop();
                if (node == end) break;
                foreach (DijkstraNode adj in node.adjacents.Keys)
                    if (!adj.visited)
                        Relax(adj, node);
            }

            MarkMap(end);
            Time = DateTime.Now - t0;
        }
        void Relax(DijkstraNode v, DijkstraNode u, DijkstraNode end = null)
        {
            if (v.distance > (u.distance + u.adjacents[v]))
            {
                v.SetDist(u.distance + u.adjacents[v]);
                v.SetFather(u);
                v.heapNode.Heapify(true);
                v.SetVisited();
            }
        }

        void MarkMap(Node node)
        {
            node.markWay = true;
            if (node.father != null)
                MarkMap(node.father);
        }
        public void Print(List<DijkstraNode> graph)
        {
            //Print for test cases 2D
            int len = (int)Math.Sqrt(graph.Count);
            int count = 0;
            int Tn = 0;
            foreach (DijkstraNode node in graph)
            {
                if (node.markWay)
                {
                    Console.ForegroundColor = ConsoleColor.Green;
                    Console.Write("o");
                    Tn++;
                }
                else
                {
                    if (node.visited)
                    {
                        Console.ForegroundColor = ConsoleColor.Red;
                        Console.Write("o");
                        Tn++;
                    }
                    else
                    {
                        Console.ForegroundColor = ConsoleColor.DarkGray;
                        Console.Write("o");
                    }
                }
                count++;
                if (count == len)
                {
                    count = 0;
                    Console.WriteLine();
                }
            }
            Console.ForegroundColor = ConsoleColor.White;
            Console.WriteLine("Total de nodos visitados: " + Tn.ToString());
            Console.WriteLine("Tiempo de ejecucion: " + Time);
            Console.WriteLine("El costo del camino total es " + END.distance);
            Console.WriteLine();
            Console.WriteLine();
            Console.WriteLine();
        }

    }
    class DijkstraA_star : Dijkstra
    {
        public DijkstraA_star(float g_weigth, float h_weigth) : base(g_weigth, h_weigth) { }


        public void Start(DijkstraNode[,] grid, DijkstraNode init, DijkstraNode end)
        {
            List<DijkstraNode> graph = new List<DijkstraNode>();
            for (int i = 0; i < grid.GetLength(0); i++)
                for (int j = 0; j < grid.GetLength(1); j++)
                    graph.Add(grid[i, j]);
            Start(graph, init, end);
        }
        public void Start(List<DijkstraNode> graph, DijkstraNode init, DijkstraNode end)
        {
            END = end;
            init.SetDist(0);
            graph[0].SetHueristicValue(graph[0].position.Distance(end.position));
            DijkstraNode root = graph[0];
            HeapNode heap = new HeapNode(root);
            foreach (DijkstraNode node in graph.GetRange(1, graph.Count - 1))
            {
                //node.SetHueristicValue(node.position.ManhatanDistance(end.position) / 2);
                node.SetHueristicValue(node.position.Distance(end.position));
                heap.Push(node);
            }
            Calculate(heap, end);

            Console.WriteLine("Caso A*:");
            Print(graph);
        }
    }
    class Node
    {
        public Dictionary<Node, float> adjacents { get; protected set; }
        public bool visited { get; protected set; }
        public Node father { get; protected set; }

        public bool markWay = false; // Use for print map

        public Node()
        {
            adjacents = new Dictionary<Node, float>();
        }

        public virtual void AddAdj(Node Node)
        {
            adjacents.Add(Node, 1);
        }
        public void SetVisited(bool visited = true)
        {
            this.visited = visited;
        }

    }
    class DijkstraNode : Node, IComparable<DijkstraNode>
    {
        float hueristicValue;
        public HeapNode heapNode { get; private set; }
        public float distance { get; private set; }
        public Point position { get; private set; }
        public DijkstraNode()
        {
            adjacents = new Dictionary<Node, float>();
            ResetValuesToDefault();
        }



        public void SetPosition(Point point)
        {
            this.position = point;
        }
        public void SetFather(Node Node)
        {
            this.father = Node;
        }
        public void SetDist(float distance)
        {
            this.distance = distance;
        }
        public void ResetValuesToDefault()
        {
            this.father = null;
            this.distance = float.MaxValue;
            this.hueristicValue = 0;
            this.heapNode = null;
            this.markWay = false;
            this.visited = false;
            //this.position = null;
        }
        public void AddAdj(Node Node, float arist)
        {
            adjacents.Add(Node, arist);
        }
        public void SetHeapNode(HeapNode heapNode)
        {
            this.heapNode = heapNode;
        }
        public void SetHueristicValue(float value)
        {
            hueristicValue = value;
        }
        public float Value(float h_weigth = 1, float g_weigth = 1)
        {
            // Value with hueristic function
            float g = distance;
            float h = hueristicValue;
            return h_weigth * h + g_weigth * g;
        }
        public override string ToString()
        {
            return ". <" + position.x.ToString() + "," + position.y.ToString() + "> value = " + Value().ToString();
        }
        public int CompareTo(DijkstraNode other)
        {
            return this.Value(Dijkstra.h_weigth,Dijkstra.g_weigth).CompareTo(other.Value(Dijkstra.h_weigth, Dijkstra.g_weigth));
            //return this.Value().CompareTo(other.Value());
        }
    }
    class HeapNode
    {

        public int size { get; private set; }

        private HeapNode father;
        private DijkstraNode value;
        private HeapNode root;
        private List<HeapNode> childs;
        private Queue<HeapNode> lastleafsByInsert;
        private Stack<HeapNode> LeafsByDelete;
        private HeapNode currentLeafByInsert;


        public HeapNode(DijkstraNode value, HeapNode father = null)
        {
            this.father = father;
            this.value = value;
            this.childs = new List<HeapNode>();


            value.SetHeapNode(this);

            if (father != null)
                this.root = father.root;
            else
            {
                this.root = this;

                this.lastleafsByInsert = new Queue<HeapNode>();
                this.LeafsByDelete = new Stack<HeapNode>();
                this.currentLeafByInsert = this;
                this.LeafsByDelete.Push(this);
                size = 1;
            }

        }
        void InsertValueInRoot(DijkstraNode value)
        {
            this.root.value = value;
            this.root.Heapify();
        }
        void InsertChild(DijkstraNode value)
        {
            this.childs.Add(new HeapNode(value, this));
            root.lastleafsByInsert.Enqueue(childs[childs.Count - 1]);
            root.LeafsByDelete.Push(childs[childs.Count - 1]);
        }
        void DeleteChild(HeapNode node)
        {
            childs.Remove(node);
        }
        public void Heapify(bool up = false)
        {
            if (!up)
            {
                if (childs.Count == 0) return;

                HeapNode minChild = childs[0];
                if (childs.Count == 2)
                    if (childs[1].value.CompareTo(minChild.value) == -1)
                        minChild = childs[1];

                if (this.value.CompareTo(minChild.value) == 1)
                {
                    DijkstraNode tempValue = minChild.value;
                    minChild.value = this.value;
                    minChild.value.SetHeapNode(minChild);

                    this.value = tempValue;
                    this.value.SetHeapNode(this);

                    minChild.Heapify();
                }
            }
            else
            {
                if (this.father != null)
                {

                    if (father.value.CompareTo(this.value) == 1)
                    {
                        DijkstraNode tempValue = this.value;
                        this.value = father.value;
                        this.value.SetHeapNode(this);

                        father.value = tempValue;
                        father.value.SetHeapNode(father);
                        father.Heapify(true);
                    }
                }
            }
        }
        public DijkstraNode Pop()
        {
            root.size--;

            DijkstraNode result = root.value;
            HeapNode leaf = root.LeafsByDelete.Pop();
            leaf.father.DeleteChild(leaf);
            InsertValueInRoot(leaf.value);
            foreach (HeapNode node in root.LeafsByDelete)
                if (node.value == result)
                    Console.WriteLine(node.value + " esta repetido");
            return result;
        }
        public void Push(DijkstraNode value)
        {
            root.size++;

            root.currentLeafByInsert.InsertChild(value);
            root.currentLeafByInsert.childs[root.currentLeafByInsert.childs.Count - 1].Heapify(true);

            if (root.currentLeafByInsert.childs.Count == 2)
                root.currentLeafByInsert = lastleafsByInsert.Dequeue();
        }
        public void Print()
        {
            foreach (HeapNode node in LeafsByDelete)
                Console.WriteLine(node.value + "\n");
            Console.WriteLine();
            Console.WriteLine();
            Console.WriteLine();
            Console.WriteLine();
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
        public float ManhatanDistance(Point point)
        {
            return Math.Abs(point.x - this.x) + Math.Abs(point.y - this.y) + Math.Abs(point.z - this.z);
        }
    }
}
