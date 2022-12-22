using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Triangle_Map;
using BaseNode;
using Agent_Space;
using Point_Map;

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

            HeapNode Q = new HeapNode(nodes[0], endNode);
            Stack<Node> A = new Stack<Node>();

            foreach (Node node in nodes.GetRange(1, nodes.Count - 1))
                Q.Push(node);

            while (Q.size > 0)
            {
                Node node = Q.Pop();
                if (node == endNode) break;
                foreach (Node adj in node.GetAdyacents())
                    if (!adj.visited)
                        Relax(adj, node);
            }
        }
        void Relax(Node v, Node u)
        {
            if (v.distance > (u.distance + v.Distance(u)))
            {
                v.SetDistance(u.distance + v.Distance(u));
                v.SetFather(u);
                v.SetVisited();
                v.heapNode.Heapify(true);
            }
        }
    }
    class HeapNode
    {
        internal Agent agent;
        internal Node endNode;
        public int size { get; private set; }

        private HeapNode father;
        private Node value;
        private HeapNode root;
        private List<HeapNode> childs;
        private Queue<HeapNode> lastleafsByInsert;
        private Stack<HeapNode> LeafsByDelete;
        private HeapNode currentLeafByInsert;


        public HeapNode(Node value, Node endNode, HeapNode father = null)
        {
            this.father = father;
            this.value = value;
            this.childs = new List<HeapNode>();
            this.endNode = endNode;

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
        void InsertValueInRoot(Node value)
        {
            this.root.value = value;
            this.root.Heapify();
        }
        void InsertChild(Node value)
        {
            this.childs.Add(new HeapNode(value, endNode, this));
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
            HeapNode leaf = root.LeafsByDelete.Pop();
            if (leaf.father != null)
                leaf.father.DeleteChild(leaf);
            InsertValueInRoot(leaf.value);

            return result;
        }
        public virtual void Push(Node value)
        {
            root.size++;

            root.currentLeafByInsert.InsertChild(value);
            root.currentLeafByInsert.childs[root.currentLeafByInsert.childs.Count - 1].Heapify(true);

            if (root.currentLeafByInsert.childs.Count == 2)
                root.currentLeafByInsert = lastleafsByInsert.Dequeue();
        }
    }

}
