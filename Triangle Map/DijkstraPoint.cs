using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Triangle_Map;

namespace Point_Map
{
    class PointNode
    {
        public static List<PointNode> CreatePointMap(List<Arist> arists, Point init, Point end, int n = 4)
        {

            List<List<PointNode>> points = new List<List<PointNode>>();
            List<PointNode> result = new List<PointNode>();

            PointNode initNode = new PointNode(init);
            result.Add(initNode);

            if (arists.Count == 0)
                return result;

            foreach (Arist arist in arists)
            {
                points.Add(new List<PointNode>());
                foreach (Point point in arist.Points(n))
                {
                    PointNode node = new PointNode(point);
                    points[points.Count - 1].Add(node);
                    result.Add(node);
                }
            }
            foreach (PointNode node in points[0])
            {
                initNode.AddAdjacent(node);
            }
            for (int i = 0; i < points.Count - 1; i++)
            {
                for (int j = 0; j < points[i].Count; j++)
                {
                    for (int k = 0; k < points[i + 1].Count; k++)
                    {
                        points[i][j].AddAdjacent(points[i + 1][k]);
                    }
                }
            }
            PointNode endNode = new PointNode(end);
            result.Add(endNode);
            foreach (PointNode node in points[points.Count - 1])
            {
                node.AddAdjacent(endNode);
            }
            return result;
        }

        Point point;
        public PointNode father { get; protected set; }
        public float distance { get; private set; }

        public List<PointNode> adjacents { get; private set; }
        public HeapPoint heapPoint { get; private set; }
        public bool visited { get; private set; }


        public PointNode(Point point)
        {
            this.point = point;
            adjacents = new List<PointNode>();
            distance = float.MaxValue;
        }
        public void AddAdjacent(PointNode node)
        {
            adjacents.Add(node);
        }
        public float CostForMoveToNode(PointNode node)
        {
            return EuclideanDistance(node);
        }
        public float EuclideanDistance(PointNode node)
        {
            return node.point.Distance(point);
        }
        public void SetDistance(float value)
        {
            distance = value;
        }
        public void SetHeapPoint(HeapPoint node)
        {
            heapPoint = node;
        }
        public void SetVisited(bool value = true)
        {
            visited = value;
        }
        public void SetFather(PointNode node)
        {
            father = node;
        }
        float Heuristic(PointNode endNode)
        {
            return EuclideanDistance(endNode);
        }
        public float Value(PointNode end)
        {
            float g = distance;
            float h = Heuristic(end);

            return g + h;
        }
        public int CompareTo(PointNode other, PointNode end)
        {
            return this.Value(end).CompareTo(other.Value(end));
        }
        public override string ToString()
        {
            return point.ToString();
        }
    }
    class DijkstraPoint
    {
        private PointNode initNode;
        private PointNode endNode;
        private List<PointNode> endPath;
        private List<PointNode> map;
        public DijkstraPoint(PointNode init, PointNode end, List<PointNode> map)
        {
            this.initNode = init;
            this.endNode = end;
            this.map = map;
        }

        public List<PointNode> GetPath()
        {
            Start();
            endPath = new List<PointNode>();
            GetPath(endNode);
            endPath.Reverse();
            return endPath;
        }
        void GetPath(PointNode node)
        {
            endPath.Add(node);
            if (node.father != null)
                GetPath(node.father);
        }

        void Start()
        {
            initNode.SetDistance(0);
            List<PointNode> nodes = map;

            PointNode root = nodes[0];

            HeapPoint Q = new HeapPoint(root, endNode);
            Stack<PointNode> A = new Stack<PointNode>();

            foreach (PointNode node in nodes.GetRange(1, nodes.Count - 1))
                Q.Push(node);

            while (Q.size > 0)
            {
                PointNode node = Q.Pop();
                if (node == endNode) break;
                foreach (PointNode adj in node.adjacents)
                    if (!adj.visited)
                        Relax(adj, node);
            }
        }
        void Relax(PointNode v, PointNode u)
        {
            if (v.distance > (u.distance + v.CostForMoveToNode(u)))
            {
                v.SetDistance(u.distance + v.CostForMoveToNode(u));
                v.SetFather(u);
                v.SetVisited();
                v.heapPoint.Heapify(true);
            }
        }
    }
    class HeapPoint
    {
        internal Agent agent;
        internal PointNode endNode;
        public int size { get; private set; }

        private HeapPoint father;
        private PointNode value;
        private HeapPoint root;
        private List<HeapPoint> childs;
        private Queue<HeapPoint> lastleafsByInsert;
        private Stack<HeapPoint> LeafsByDelete;
        private HeapPoint currentLeafByInsert;


        public HeapPoint(PointNode value, PointNode endNode, HeapPoint father = null)
        {
            this.father = father;
            this.value = value;
            this.childs = new List<HeapPoint>();
            this.endNode = endNode;

            value.SetHeapPoint(this);

            if (father != null)
                this.root = father.root;
            else
            {
                this.root = this;

                this.lastleafsByInsert = new Queue<HeapPoint>();
                this.LeafsByDelete = new Stack<HeapPoint>();
                this.currentLeafByInsert = this;
                this.LeafsByDelete.Push(this);
                size = 1;
            }

        }
        void InsertValueInRoot(PointNode value)
        {
            this.root.value = value;
            this.root.Heapify();
        }
        void InsertChild(PointNode value)
        {
            this.childs.Add(new HeapPoint(value, endNode, this));
            root.lastleafsByInsert.Enqueue(childs[childs.Count - 1]);
            root.LeafsByDelete.Push(childs[childs.Count - 1]);
        }
        void DeleteChild(HeapPoint node)
        {
            childs.Remove(node);
        }
        public void Heapify(bool up = false)
        {
            if (!up)
            {
                if (childs.Count == 0) return;

                HeapPoint minChild = childs[0];
                if (childs.Count == 2)
                    if (childs[1].value.CompareTo(minChild.value, endNode) == -1)
                        minChild = childs[1];

                if (this.value.CompareTo(minChild.value, endNode) == 1)
                {
                    PointNode tempValue = minChild.value;
                    minChild.value = this.value;
                    minChild.value.SetHeapPoint(minChild);

                    this.value = tempValue;
                    this.value.SetHeapPoint(this);

                    minChild.Heapify();
                }
            }
            else
            {
                if (this.father != null)
                {

                    if (father.value.CompareTo(this.value, endNode) == 1)
                    {
                        PointNode tempValue = this.value;
                        this.value = father.value;
                        this.value.SetHeapPoint(this);

                        father.value = tempValue;
                        father.value.SetHeapPoint(father);
                        father.Heapify(true);
                    }
                }
            }
        }
        public PointNode Pop()
        {
            root.size--;

            PointNode result = root.value;
            HeapPoint leaf = root.LeafsByDelete.Pop();
            if (leaf.father != null)
                leaf.father.DeleteChild(leaf);
            InsertValueInRoot(leaf.value);

            return result;
        }
        public virtual void Push(PointNode value)
        {
            root.size++;

            root.currentLeafByInsert.InsertChild(value);
            root.currentLeafByInsert.childs[root.currentLeafByInsert.childs.Count - 1].Heapify(true);

            if (root.currentLeafByInsert.childs.Count == 2)
                root.currentLeafByInsert = lastleafsByInsert.Dequeue();
        }
    }

}
