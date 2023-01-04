using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Triangle_Map;
using BaseNode;
using Agent_Space;
using Point_Map;
using UnityEngine;

namespace DijkstraSpace
{
    class Dijkstra
    {
        private Node initNode;
        private Node endNode;
        private List<Node> endPath;

        private Node[] nodes;
        public Dijkstra(Node init, Node end, Node[] nodes)
        {
            this.initNode = init;
            this.endNode = end;
            this.nodes = nodes;
            endPath = new List<Node>();
        }

        public List<Node> GetPath()
        {
            Start();
            GetPath(endNode);
            endPath.Reverse();
            return endPath;
        }
        void GetPath(Node node)
        {
            endPath.Add(node);
            if (node.father != null)
                GetPath(node.father);
        }

        void Start()
        {
            initNode.SetDistance(0);

            List<Node> nodes = this.nodes.ToList<Node>();

            Heap Q = new Heap(nodes[0]);
            Stack<Node> A = new Stack<Node>();

            foreach (Node node in nodes.GetRange(1, nodes.Count - 1))
                Q.Push(node);

            while (Q.size > 0)
            {

                Node node = Q.Pop();
                node.SetVisited();
                if (node == endNode)
                    break;

                foreach (Node adj in node.GetAdyacents())
                    if (!adj.visited)
                        Relax(adj, node);
            }
        }
        void Relax(Node v, Node u)
        {
            if (v.distance > (u.distance + u.Distance(v)))
            {
                v.SetDistance(u.distance + u.Distance(v));
                v.SetFather(u);
                v.heapNode.Heapify(true);
            }
        }
    }
    public class Heap
    {
        internal Agent agent;
        public int size { get; private set; }

        private Heap father;
        private Node value;
        private Heap root;
        private List<Heap> childs;
        private Queue<Heap> lastleafsByInsert;
        private Stack<Heap> LeafsByDelete;
        private Heap currentLeafByInsert;


        public Heap(Node value, Heap father = null)
        {
            this.father = father;
            this.value = value;
            this.childs = new List<Heap>();

            value.SetHeapNode(this);

            if (father != null)
                this.root = father.root;
            else
            {
                this.root = this;

                this.lastleafsByInsert = new Queue<Heap>();
                this.LeafsByDelete = new Stack<Heap>();
                this.currentLeafByInsert = this;
                this.LeafsByDelete.Push(this);
                size = 1;
            }

        }
        void InsertValueInRoot(Node value)
        {
            this.root.value = value;
            this.root.Heapify();
        }
        void InsertChild(Node value)
        {
            this.childs.Add(new Heap(value, this));
            root.lastleafsByInsert.Enqueue(childs[childs.Count - 1]);
            root.LeafsByDelete.Push(childs[childs.Count - 1]);
        }
        void DeleteChild(Heap node)
        {
            childs.Remove(node);
        }
        public void Heapify(bool up = false)
        {
            if (!up)
            {
                if (childs.Count == 0) return;

                Heap minChild = childs[0];
                if (childs.Count == 2)
                    if (childs[1].value.CompareTo(minChild.value) == -1)
                        minChild = childs[1];

                if (this.value.CompareTo(minChild.value) == 1)
                {
                    Node tempValue = minChild.value;
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
                        Node tempValue = this.value;
                        this.value = father.value;
                        this.value.SetHeapNode(this);

                        father.value = tempValue;
                        father.value.SetHeapNode(father);
                        father.Heapify(true);
                    }
                }
            }
        }
        public Node Pop()
        {
            root.size--;

            Node result = root.value;

            if (root.size != 0)
            {
                Heap leaf = root.LeafsByDelete.Pop();
                if (leaf.father != null)
                    leaf.father.DeleteChild(leaf);

                InsertValueInRoot(leaf.value);
                return result;
            }
            else
            {
                root.value = null;
                return result;
            }
        }
        public virtual void Push(Node value)
        {
            root.size++;
            if (root.value == null)
            {
                root.value = value;
                return;
            }

            root.currentLeafByInsert.InsertChild(value);
            root.currentLeafByInsert.childs[root.currentLeafByInsert.childs.Count - 1].Heapify(true);

            if (root.currentLeafByInsert.childs.Count == 2)
                root.currentLeafByInsert = lastleafsByInsert.Dequeue();
        }
    }

}
