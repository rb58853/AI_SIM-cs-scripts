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
        private Dictionary<MapNode, bool> grupalDic;
        private ICollection<MapNode> grupalList;


        private bool grupalMove;
        private Node[] nodes;
        public Dijkstra(Node init, Node end, Node[] nodes, ICollection<MapNode> grupal = null)
        {
            if (grupal == null)
                grupalMove = false;
            else
            {
                this.grupalDic = new Dictionary<MapNode, bool>();
                grupalList = new List<MapNode>();

                foreach (MapNode node in grupal)
                    if (!this.grupalDic.ContainsKey(node))
                        this.grupalDic.Add(node, false);
                grupalMove = true;
            }

            this.initNode = init;
            this.endNode = end;
            this.nodes = nodes;
            endPath = new List<Node>();
        }

        public List<Node> GetPath(bool stop = true)
        {
            Start(stop);
            if (grupalMove)
            {
                return GetPathGrupal();
            }
            else
            {
                GetPath(endNode);
                endPath.Reverse();
                return endPath;
            }
        }
        List<Node> GetPathGrupal()
        {
            List<Node> result = new List<Node>();
            Queue<Node> q = new Queue<Node>();
            foreach (Node node in grupalList)
                q.Enqueue(node);

            while (q.Count > 0)
            {
                Node node = q.Dequeue();
                result.Add(node);
                // (node as MapNode).triangle.draw(Color.yellow);

                if (node.father != null && !node.father.visitedAsGrupal)
                {
                    node.father.visitedAsGrupal = true;
                    q.Enqueue(node.father);
                }
            }
            return result;
        }
        void GetPath(Node node)
        {
            endPath.Add(node);
            if (node.father != null)
                GetPath(node.father);
        }

        void Start(bool stop = true)
        {

            // DateTime t0 = DateTime.Now;
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

                if (stop && node == endNode && !grupalMove)
                    break;

                if (grupalMove)
                {
                    if (grupalDic.ContainsKey((node as MapNode).origin))
                    {
                        grupalDic.Remove((node as MapNode).origin);
                        grupalList.Add(node as MapNode);
                        node.visitedAsGrupal = true;
                        if (grupalDic.Count == 0)
                            break;
                    }
                    while (grupalDic.ContainsKey((node as MapNode).origin))
                    {
                        grupalDic.Remove((node as MapNode).origin);
                        if (grupalDic.Count == 0)
                            break;
                    }
                }

                foreach (Node adj in node.GetAdyacents())
                    if (!adj.visited)
                        Relax(adj, node);
            }
            // Debug.Log("el Dijkstra tardo: " + (DateTime.Now - t0));
        }
        void Relax(Node v, Node u)
        {
            if (v.distance > (u.distance + u.Distance(v)))
            {
                v.SetDistance(u.distance + u.Distance(v));
                v.SetFather(u);
                try { v.heapNode.Heapify(true); }
                catch { Debug.Log("No encontro el heap Node"); }
            }
        }
    }
    public class Heap
    {
        internal Agent agent;
        public int size { get; private set; }
        public Node next { get => value; }
        private Heap father;
        private Node value;
        private Heap root;
        private List<Heap> childs;
        private Queue<Heap> lastleafsByInsert;
        private Stack<Heap> LeafsByDelete;
        private Heap currentLeafByInsert;


        public Heap(Node value = null, Heap father = null)
        {
            this.father = father;
            this.value = value;
            this.childs = new List<Heap>();

            if (value != null)
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
                if (value != null)
                    size = 1;
                else
                    size = 0;
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
                root.value.SetHeapNode(root);
                return;
            }

            root.currentLeafByInsert.InsertChild(value);
            root.currentLeafByInsert.childs[root.currentLeafByInsert.childs.Count - 1].Heapify(true);

            if (root.currentLeafByInsert.childs.Count == 2)
                root.currentLeafByInsert = lastleafsByInsert.Dequeue();
        }
    }

}
