using System;
using System.Collections.Generic;
using System.Text;
using Triangle_Map;

namespace Dijkstra_Algorithm
{
    class Dijkstra
    {
        private MapNode initNode;
        private MapNode endNode;
        private Agent agent;
        private Map map;
        private List<MapNode> endWay;
        public Dijkstra(Map map, MapNode init, MapNode end, Agent agent)
        {
            this.map = map;
            this.initNode = init;
            this.endNode = end;
            this.agent = agent;
        }

        public List<MapNode> GetWay()
        {
            Start();
            endWay.Reverse();
            return endWay;
        }
        void GetWay(MapNode node)
        {
            endWay.Add(node);
            if (node.father != null)
                GetWay(node.father[agent]);
        }

        void Start()
        {
            initNode.SetDistance(0,agent);
            List<MapNode> nodes = map.nodes;

            MapNode root = nodes[0];

            HeapNode Q = new HeapNode(root,agent,endNode);
            Stack<MapNode> A = new Stack<MapNode>();

            foreach (MapNode node in nodes.GetRange(1, nodes.Count - 1))
                Q.Push(node);

            while (Q.size > 0)
            {
                MapNode node = Q.Pop();
                if (node == endNode) break;
                foreach(MapNode adj in node.adjacents.Keys)
                    if (!adj.visited[agent])
                        Relax(adj, node);
            }
        }
        void Relax(MapNode v, MapNode u)
        {
            if (v.distance[agent] > (u.distance[agent] + v.CostForMoveToNodeByAgent(u,agent)))
            {
                v.SetDistance(u.distance[agent] + v.CostForMoveToNodeByAgent(u, agent),agent);
                v.SetFather(u,agent);
                v.SetVisited(agent);
                v.heapNodes[agent].Heapify(true);
            }
        }
    }

    class HeapNode
    {
        internal Agent agent;
        internal MapNode endNode;
        public int size { get; private set; }

        private HeapNode father;
        private MapNode value;
        private HeapNode root;
        private List<HeapNode> childs;
        private Queue<HeapNode> lastleafsByInsert;
        private Stack<HeapNode> LeafsByDelete;
        private HeapNode currentLeafByInsert;


        public HeapNode(MapNode value, Agent agent, MapNode endNode, HeapNode father = null)
        {
            this.father = father;
            this.value = value;
            this.childs = new List<HeapNode>();
            this.agent = agent;
            this.endNode = endNode;

            value.SetHeapNode(this, agent);

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
        void InsertValueInRoot(MapNode value)
        {
            this.root.value = value;
            this.root.Heapify();
        }
        void InsertChild(MapNode value)
        {
            this.childs.Add(new HeapNode(value, agent, endNode, this));
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
                    if (childs[1].value.CompareTo(minChild.value, agent, endNode) == -1)
                        minChild = childs[1];

                if (this.value.CompareTo(minChild.value, agent, endNode) == 1)
                {
                    MapNode tempValue = minChild.value;
                    minChild.value = this.value;
                    minChild.value.SetHeapNode(minChild, agent);

                    this.value = tempValue;
                    this.value.SetHeapNode(this, agent);

                    minChild.Heapify();
                }
            }
            else
            {
                if (this.father != null)
                {

                    if (father.value.CompareTo(this.value, agent, endNode) == 1)
                    {
                        MapNode tempValue = this.value;
                        this.value = father.value;
                        this.value.SetHeapNode(this, agent);

                        father.value = tempValue;
                        father.value.SetHeapNode(father, agent);
                        father.Heapify(true);
                    }
                }
            }
        }
        public MapNode Pop()
        {
            root.size--;

            MapNode result = root.value;
            HeapNode leaf = root.LeafsByDelete.Pop();
            leaf.father.DeleteChild(leaf);
            InsertValueInRoot(leaf.value);

            return result;
        }
        public void Push(MapNode value)
        {
            root.size++;

            root.currentLeafByInsert.InsertChild(value);
            root.currentLeafByInsert.childs[root.currentLeafByInsert.childs.Count - 1].Heapify(true);

            if (root.currentLeafByInsert.childs.Count == 2)
                root.currentLeafByInsert = lastleafsByInsert.Dequeue();
        }
    }
}
