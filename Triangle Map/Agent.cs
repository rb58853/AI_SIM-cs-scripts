using System;
using System.Collections.Generic;
using BaseNode;
using DijkstraSpace;
using Point_Map;
using Triangle_Map;
using UnityEngine;

namespace Agent_Space
{
    public class Agent
    {
        public string name { get; private set; }
        public List<MapNode> ocupedNodes { get; private set; }
        public List<Triangle> ocupedTriangles { get => UcupedTriangles(); }
        List<Triangle> UcupedTriangles()
        {
            List<Triangle> result = new List<Triangle>();
            foreach (MapNode node in ocupedNodes)
                result.Add(node.triangle);
            return result;
        }
        public float radius { get; private set; }

        public MapNode currentNode { get; private set; }
        public MapNode endMapNodeCurrent { get; private set; }
        public MapNode initMapNodeCurrent { get; private set; }
        public Point position { get; private set; }
        public PointNode currentPosition { get; private set; }
        public PointNode nextPosition { get; private set; }

        public List<MapNode> triangleList { get; private set; }
        private PointPath pointPath;
        public Point destination { get; private set; }
        public PointNode endPointNode { get; private set; }

        public Stack<Point> visualPath { get; private set; }
        public bool inMove { get; /*private*/ set; }
        internal bool metaPath = false;
        /// <summary> Compatibility of this Agent whit a material.</summary>
        public Dictionary<Material, float> compatibility;
        public bool grupalMove { get; private set; }

        public Agent(float radius, string name = "agent")
        {
            pointPath = new PointPath(this);
            this.name = name;
            compatibility = new Dictionary<Material, float>();
            visualPath = new Stack<Point>();
            ocupedNodes = new List<MapNode>();
            this.radius = radius;
            inMove = false;
        }
        public void setGrup(bool inGrup = true)
        {
            if (inGrup)
            {
                grupalMove = true;
                Environment.Interactive.grup.Add(this);
            }
            else
            {
                grupalMove = false;
                if (Environment.Interactive.grup.Contains(this))
                    Environment.Interactive.grup.Remove(this);
            }
        }
        public void SetCompatibility(Material material, float value)
        {
            if (compatibility.ContainsKey(material))
                compatibility[material] = value;
            else
                compatibility.Add(material, value);
        }
        public void searchCurrentNode()
        {
            foreach (Node node in Environment.map.nodes)
                if ((node as MapNode).triangle.PointIn(position))
                {
                    currentNode = node as MapNode;
                    ocupedNodes.Add(currentNode);
                    currentNode.AddAgent(this);
                    pointPath.SetCurrentTriangle(currentNode);
                    SetOcupedFromPosition(Environment.ocupedArea);
                    break;
                }
            Agent.Collision(position, position, this, currentNode);
        }
        public void setPosition(Point point)
        {
            position = point;
            currentPosition = new PointNode(position);
        }
        public void SetOcupedFromPosition(float extendArea = 1)
        {
            Queue<MapNode> ocuped = new Queue<MapNode>();

            ocuped.Enqueue(currentNode);

            if (!ocupedNodes.Contains(currentNode))
            {
                ocupedNodes.Add(currentNode);
                currentNode.AddAgent(this);
            }

            for (int i = 0; i < ocupedNodes.Count; i++)
            {
                MapNode node = ocupedNodes[i];
                if (node != currentNode)
                {
                    if (position.DistanceToTriangle(node.triangle) > radius * extendArea)
                    {
                        node.RemoveAgent(this);
                        ocupedNodes.Remove(node);
                        i -= 1;
                    }
                    else
                        ocuped.Enqueue(node);
                }
            }

            while (ocuped.Count > 0)
            {
                MapNode node = ocuped.Dequeue();

                foreach (MapNode adj in node.adjacents.Keys)
                    if (position.DistanceToTriangle(adj.triangle) < radius + extendArea)
                        if (!ocupedNodes.Contains(adj))
                        {
                            ocuped.Enqueue(adj);
                            ocupedNodes.Add(adj);
                            adj.AddAgent(this);
                        }
            }
            ///Tambien cada esta frecuencia separar a los agentes    
            Agent.Collision(position, position, this, currentNode);
        }
        Tuple<MapNode[], MapNode, MapNode> LocalMap(Point endPoint)
        {
            return BFS(endPoint);
        }
        Tuple<MapNode[], MapNode, MapNode> BFS(Point endPoint)
        {
            bool endWasFound = false;
            int countAfterFound = Environment.bfsArea;

            List<MapNode> localMap = new List<MapNode>();

            Dictionary<MapNode, MapNode> r = new Dictionary<MapNode, MapNode>();
            List<MapNode> visited = new List<MapNode>();//Mejorar eficiencia de esto, no me gusta

            Queue<MapNode> q = new Queue<MapNode>();

            MapNode end = null;

            currentNode = currentNode.origin;
            r.Add(currentNode, new MapNode(currentNode, this, endPoint));
            visited.Add(currentNode);

            localMap.Add(r[currentNode]);


            if (currentNode.triangle.PointIn(endPoint))
                ///If endPoint is in Current node, return only current node as path, init and endNode
                return new Tuple<MapNode[], MapNode, MapNode>(localMap.ToArray(), r[currentNode], r[currentNode]);

            q.Enqueue(currentNode);

            while (q.Count > 0)
            {
                if (endWasFound == true)
                {
                    countAfterFound--;
                    if (countAfterFound <= 0) break;
                }

                MapNode n = q.Dequeue();
                foreach (MapNode adj in n.adjacents.Keys)
                {
                    Arist aristClone = n.adjacents[adj].Clone();

                    if (!visited.Contains(adj))
                    {
                        visited.Add(adj);
                        MapNode temp = new MapNode(adj, this, endPoint);

                        if (adj.triangle.PointIn(endPoint))
                        {
                            end = temp;
                            endWasFound = true;
                        }

                        r.Add(adj, temp);
                        r[n].AddAdjacent(temp, aristClone);
                        temp.AddAdjacent(r[n], aristClone);
                        aristClone.AddTriangle(r[n]);
                        aristClone.AddTriangle(temp);
                        q.Enqueue(adj);

                        localMap.Add(temp);
                    }
                    else
                    {
                        if (!r[n].adjacents.ContainsKey(r[adj]))
                        {
                            r[n].AddAdjacent(r[adj], aristClone);
                            r[adj].AddAdjacent(r[n], aristClone);
                            aristClone.AddTriangle(r[n]);
                            aristClone.AddTriangle(r[adj]);
                        }
                    }
                }
            }

            if (end == null)
                localMap = new List<MapNode>();

            return new Tuple<MapNode[], MapNode, MapNode>(localMap.ToArray(), r[currentNode], end);
        }
        DateTime initCreatePath;
        public MapNode[] GetTrianglePath(Point endPoint, bool push = true)
        {
            Tuple<MapNode[], MapNode, MapNode> localMap = LocalMap(endPoint);
            Node[] nodes = localMap.Item1;
            Node init = localMap.Item2;
            Node end = localMap.Item3;

            endMapNodeCurrent = end as MapNode;
            initMapNodeCurrent = init as MapNode;

            if (end == null)
                return null;

            PointNode endPointNode = new PointNode(endPoint, agent: this);
            endPointNode.SetDistance(0);

            PointNode tempPosition = new PointNode(position);
            DateTime t0 = DateTime.Now;
            if (Environment.metaheuristic)
                if (Metaheuristic.Path(initMapNodeCurrent, endMapNodeCurrent, endPointNode, tempPosition))
                {
                    Debug.Log("En encontrar el camino meta demora " + (DateTime.Now - t0));

                    metaPath = true;
                    if (!endPointNode.triangles.Contains(endMapNodeCurrent))
                        endPointNode.AddTriangle(endMapNodeCurrent);
                    this.endPointNode = endPointNode;

                    tempPosition.AddTriangle(initMapNodeCurrent);

                    this.pointPath.pushMetaMap(tempPosition);
                    return null;
                }
            initCreatePath = DateTime.Now;
            // this.currentPosition.AddTriangle(currentNode);
            endPointNode.AddTriangle(endMapNodeCurrent);
            this.endPointNode = endPointNode;

            Dijkstra dijkstra = new Dijkstra(end, init, nodes);
            if (grupalMove)
            {
                List<MapNode> grup = new List<MapNode>();
                foreach (Agent agent in Environment.Interactive.grup)
                    grup.Add(agent.currentNode.origin);
                dijkstra = new Dijkstra(end, init, nodes, grup);
            }
            List<Node> path = dijkstra.GetPath();

            pointPath.PushCurrenTriangle(initMapNodeCurrent);
            // if (!grupalMove)
            DilatePath();
            triangleList = tools.ToListAsMapNode(path);

            MapNode[] result = tools.ToArrayAsMapNode(path);
            return result;

            void DilatePath()
            {
                int dilate = Environment.trianglePathDilatation;

                while (dilate > 0)
                {
                    dilate--;
                    MapNode temp = path[path.Count - 1] as MapNode;
                    Node[] array = path.ToArray();

                    path.Remove(temp);

                    foreach (MapNode node in array)
                        foreach (MapNode adj in node.adjacents.Keys)
                            if (!path.Contains(adj) && adj != temp)
                                path.Add(adj);

                    path.Add(temp);
                }

                // foreach (MapNode node in path.ToArray())
                //     if (node != path[0] && node != path[path.Count - 1])
                //         if (OnlyOneAdj(node))
                //             path.Remove(node);

                // bool OnlyOneAdj(MapNode triangle)
                // {
                //     int countAdj = 0;
                //     foreach (MapNode adj in triangle.adjacents.Keys)
                //         if (path.Contains(adj))
                //             countAdj++;

                //     return countAdj <= 1;
                // }
            }
        }
        public PointNode[] GetPointPath(Point endPoint)
        {
            MapNode[] tPath = GetTrianglePath(endPoint);
            if (metaPath == true) return null;

            if (tPath == null)
            {
                this.pointPath.PushPointMap(new PointNode[1] { new PointNode(position) });
                return new PointNode[1] { new PointNode(position) };///Debugguer
            }

            float density = Environment.densityPath;
            float mCost = currentNode.MaterialCost(this);

            // PointNode endNode = new PointNode(endPoint, inArist: false, agent: this);
            // endNode.AddTriangle(endMapNodeCurrent);
            // endNode.AddTriangle(triangleList[0]);
            // endPointNode = endNode;

            // List<PointNode> mapPoints = PointNode.Static.CreatePointMap(endNode, position, this, density, mCost);
            List<PointNode> mapPoints = PointNode.Static.CreatePointMap(this.endPointNode, position, this, density, mCost);

            ///MapPoints[0] = initNode
            ///MapPoints[MapPoints.Count-1] = endNode
            Dijkstra dijkstra = new Dijkstra(mapPoints[0], mapPoints[mapPoints.Count - 1], mapPoints.ToArray());
            List<Node> pointPath = dijkstra.GetPath(false);

            // currentPositionDynamic = mapPoints[mapPoints.Count - 1];///Nuevo
            initMapNodeCurrent.triangle.draw(Color.black);
            // this.pointPath.PushPointMap(mapPoints[0], mapPoints[mapPoints.Count - 1], initMapNodeCurrent);
            // currentPosition = mapPoints[mapPoints.Count - 1];
            endPointNode = mapPoints[0];
            this.pointPath.PushPointMap(endPointNode, mapPoints[mapPoints.Count - 1], initMapNodeCurrent);

            Debug.Log("En crear el camino demora " + (DateTime.Now - initCreatePath));
            return tools.ToArrayAsPointNode(pointPath);///Debugguer
        }
        public void SetPointPath(Point point)
        {
            if (grupalMove)
            {
                if (Environment.Interactive.allGroupInMove == false)
                    foreach (Agent agent in Environment.Interactive.grup)
                        agent.inMove = true;
                Environment.Interactive.allGroupInMove = true;
                Environment.Interactive.countInStop = 0;
            }
            metaPath = false;
            pointPath.clear();
            pointPath.Move();
            destination = point;
            GetPointPath(point);
            if (metaPath == false)
            {
                PointNode tempPosition = new PointNode(position);
                foreach (MapNode triangle in currentPosition.triangles)
                    tempPosition.AddTriangle(triangle);
                Metaheuristic.Proccess(endMapNodeCurrent, point, triangleList, endPointNode, currentNode, tempPosition);
                currentPosition = tempPosition;
            }
            NextPoint();
        }
        void setInMoveGrupal()
        {
            if (!grupalMove) return;
            if (position.Distance(destination) < 0.6f * radius * Environment.Interactive.countInStop)
            {
                inMove = false;
                Environment.Interactive.allGroupInMove = false;
                Environment.Interactive.countInStop += 1;
            }

            // foreach (Agent agent in Environment.Interactive.grup)
            //     if (agent.inMove == false)
            //     {
            // if (position.Distance(agent.position, false) < 3 * (radius + agent.radius))
            // {
            //     inMove = false;
            //     Environment.Interactive.allGroupInMove = false;
            //     Environment.Interactive.countInStop += 1;
            //     break;
            // }
            //     }
        }
        void DynamicSetPoint()
        {
            // if (nextPosition.point.Distance(position) < 0.01f) return;

            float dist = radius * Environment.viewLenAgent;
            Point pointDest = position + Point.VectorUnit(position, nextPosition.point) * dist;

            Tuple<bool, Agent> collision =
            Collision(position, pointDest, this, ocupedNodes, multArea: 1,
             maxDistance: Environment.distanceAnalizeCollision);
            if (collision.Item1)
                NextPoint(true);
        }

        int countMoves = 1;
        public void NextMove(int n = 1)
        {
            for (int i = 0; i < n; i++)
                NextMoveBasic();

            countMoves--;
            if (countMoves <= 0)
            {
                countMoves = 1;
                SetOcupedFromPosition(Environment.ocupedArea);
            }
        }

        int freq = Environment.freqReview;
        private int stopCount = Environment.stopCountForEmpty;
        void NextMoveBasic()
        {
            if (pointPath.stop && inMove)
            {
                pointPath.EmptyMove();
                if (!pointPath.stop)
                {
                    NextPoint();
                    if (pointPath.stop)
                    {
                        stopCount--;
                        if (stopCount <= 0)
                        {
                            SetPointPath(position);/// Quitarse esto en algun momento
                            // inMove = false;
                        }
                    }
                    else
                        stopCount = Environment.stopCountForEmpty;
                }
            }

            if (inMove && !pointPath.stop)
            {
                if (freq <= 0 && !pointPath.stop && inMove)
                {
                    freq = Environment.freqReview;
                    DynamicSetPoint();
                }
                freq--;

                Point temp = position;
                int count = visualPath.Count;
                if (visualPath.Count == 0) NextPoint();
                try
                {
                    if (inMove && !pointPath.stop)
                    {
                        position = visualPath.Pop();
                        // currentPosition = new PointNode(position);///Nuevo
                    }
                }
                catch { Debug.Log("Error: la pila tiene " + visualPath.Count + " elementos y esta intentando hacer Pop()."); }
                // if (temp.Distance(position, false) > 0.6f)
                // {
                //     Debug.Log("Salto desde " + temp + " hasta " + position +
                //     "  cuando el count del visual path es " + count);
                //     PointNode.Static.DrawTwoPoints(temp, position, Color.green);
                // }
            }
        }

        void NextPoint(bool onCollision = false)
        {
            visualPath.Clear();
            if (!pointPath.empty)
            {
                inMove = true;
                setInMoveGrupal();
                nextPosition = pointPath.Pop(onCollision);
                currentPosition = pointPath.currentPoint;
                SetCurrentTriangle();

                if (nextPosition == currentPosition)
                {
                    visualPath.Push(currentPosition.point);
                    // NextMoveBasic();
                    return;
                }
                // float cost = currentPosition.adjacents[nextPosition] * 25;
                float cost = currentNode.MaterialCost(this) * Environment.densityVisualPath;
                List<Point> temp = new Arist(currentPosition.point, nextPosition.point).ToPoints(cost);
                for (int i = temp.Count - 1; i >= 0; i--)
                {
                    if (i > 0)
                        if (temp[i].Distance(temp[i - 1], false) > 0.2f)
                            Debug.Log("se creo un punto largo que no va");
                    visualPath.Push(temp[i]);
                }

                // Debug.Log("currentPosition = " + currentPosition + "   nextPosition = " + nextPosition +
                // "Count de path visual " + visualPath.Count);

                // NextMoveBasic();
            }
            else
            {
                Environment.Interactive.countInStop += 3;
                inMove = false;
            }
        }
        void SetCurrentTriangle()
        {
            currentNode = pointPath.currentTriangle;
        }
        public static Tuple<bool, Agent> Collision(Point node1, Point node2, Agent agent, MapNode mapNode, float multArea = 1,
        float maxDistance = 500f)
        {
            Point l1 = node1;
            Point l2 = node2;
            if (!Environment.exactCollision)
            {
                l1 = l1 + Point.VectorUnit(l2 - l1) * agent.radius;
                if (l1.Distance(node1, false) > l2.Distance(node1, false))
                    l1 = l2;
            }
            float epsilon = 0.005f;

            Tuple<bool, Agent> result = new Tuple<bool, Agent>(false, null);

            foreach (Agent agentObstacle in mapNode.origin.agentsIn)
            {
                if (agentObstacle == agent) continue;
                if (agentObstacle.position.Distance(agent.position) > (agent.radius + agentObstacle.radius) * maxDistance) continue;

                if (agentObstacle.position.DistanceToSegment(l1, l2) <= (agent.radius + agentObstacle.radius) * multArea + epsilon)
                    /// Collision
                    if (result.Item2 == null ||
                        agentObstacle.position.Distance(node1, false) <
                        result.Item2.position.Distance(node1, false))///mas cercano

                        result = new Tuple<bool, Agent>(true, agentObstacle);
            }
            if (result.Item1)
            {
                float distance = result.Item2.position.Distance(agent.position, false);
                float radius = result.Item2.radius + agent.radius;
                if (distance <= radius * multArea + epsilon)
                {
                    ///Choque
                    Point vector = Point.VectorUnit(result.Item2.position, agent.position) * ((radius - distance) / 2 + epsilon);

                    agent.position = agent.position + vector * 1.1f;
                    Collision(node1, node2, agent, mapNode);/// Es lo que debe, pero se puede poner muy lento
                }
            }

            return result;
        }
        public static Tuple<bool, Agent> Collision(Point node1, Point node2, Agent agent, ICollection<MapNode> mapNodes, float multArea = 1,
        float maxDistance = 500f)
        {
            foreach (MapNode node in mapNodes)
            {
                Tuple<bool, Agent> collision = Collision(node1, node2, agent, node, multArea, maxDistance);
                if (collision.Item1)
                    return collision;
            }
            return new Tuple<bool, Agent>(false, null);
        }
        public override string ToString() { return name; }
        internal class tools
        {
            internal static MapNode[] ToArrayAsMapNode(List<Node> list)
            {
                MapNode[] result = new MapNode[list.Count];
                for (int i = 0; i < list.Count; i++)
                    result[i] = list[i] as MapNode;
                return result;
            }
            internal static List<MapNode> ToListAsMapNode(List<Node> list)
            {
                List<MapNode> result = new List<MapNode>();
                for (int i = 0; i < list.Count; i++)
                    result.Add(list[i] as MapNode);
                return result;
            }
            internal static PointNode[] ToArrayAsPointNode(List<Node> list)
            {
                PointNode[] result = new PointNode[list.Count];
                for (int i = 0; i < list.Count; i++)
                    result[i] = list[i] as PointNode;
                return result;
            }
        }
        internal static class Metaheuristic
        {
            public static Dictionary<Triangle, Dictionary<MapNode, MapNode>> origins;

            public static bool Path(MapNode initTriangle, MapNode endTriangle,
            PointNode endPoint, PointNode initPoint)
            {
                if (origins == null || initPoint == null) return false;
                initPoint.adjacents.Clear();
                initPoint.SetDistance(float.MaxValue);

                foreach (Triangle triangle in endTriangle.origin.triangle.trianglesSub)
                {
                    if (triangle.PointIn(endPoint.point))
                    {
                        if (origins.ContainsKey(triangle))
                            if (origins[triangle].ContainsKey(initTriangle.origin))
                            {
                                Dictionary<MapNode, MapNode> originsNodes = origins[triangle];

                                initTriangle = originsNodes[initTriangle.origin];
                                endTriangle = originsNodes[endTriangle.origin];

                                foreach (Arist arist in initTriangle.adjacents.Values)
                                    foreach (PointNode point in arist.points)
                                    {
                                        initPoint.AddAdjacent(point, initTriangle.MaterialCost(new Agent(1)));
                                        PointNode.Static.DrawTwoPoints(initPoint.point, point.point, Color.cyan);
                                    }

                                foreach (Arist arist in endTriangle.adjacents.Values)
                                    foreach (PointNode point in arist.points)
                                        point.AddAdjacent(endPoint, endTriangle.MaterialCost(new Agent(1)));

                                return true;
                            }
                        return false;
                    }
                }
                return false;
            }
            public static void Proccess(MapNode endTriangle, Point endPoint,
            List<MapNode> nodes, PointNode endPointNode, MapNode initMapNode, PointNode initPointNode)
            {
                if (!Environment.metaheuristic) return;
                foreach (Triangle triangle in endTriangle.origin.triangle.trianglesSub)
                {
                    if (triangle.PointIn(endPoint))
                    {
                        Merge(triangle, nodes, endPointNode, endTriangle, initPointNode);
                        move.triangle = triangle;
                        triangle.draw(Color.green);
                        return;
                    }
                }

            }
            static void Merge(Triangle triangle, List<MapNode> inNodes,
            PointNode endPointNode, MapNode endMapNode, PointNode initPointNode)
            {
                if (origins == null)
                {
                    origins = new Dictionary<Triangle, Dictionary<MapNode, MapNode>>();
                }
                List<MapNode> tempInputNodes = new List<MapNode>();

                MapNode initMapNode = inNodes[inNodes.Count - 1];

                if (origins.ContainsKey(triangle))
                {
                    Dictionary<MapNode, MapNode> originsNodes = origins[triangle];

                    foreach (MapNode node in inNodes)
                        tempInputNodes.Add(node);

                    foreach (MapNode node in inNodes)
                        if (originsNodes.ContainsKey(node.origin))
                        {
                            tempInputNodes.Remove(node);
                            if (node == endMapNode)
                            {
                                MapNode localEnd = originsNodes[node.origin];

                                foreach (Arist arist in localEnd.adjacents.Values)
                                    foreach (PointNode point in arist.points)
                                        ///Crear adyacencia hacia el nuevo nodo final
                                        point.AddAdjacent(endPointNode);
                            }
                            if (node == initMapNode)
                            {
                                // initPointNode.adjacents.Clear();
                                // MapNode localInit = originsNodes[node.origin];
                                // foreach (Arist arist in localInit.adjacents.Values)
                                //     foreach (PointNode point in arist.points)
                                //     {
                                //         ///Crear adyacencia desde el nodo inicio
                                //         initPointNode.AddAdjacent(point);
                                //         Point p = new Point(-0.05f, 0, 0);
                                //         PointNode.Static.DrawTwoPoints(initPointNode.point, point.point, Color.magenta);
                                //     }
                            }
                        }

                    foreach (MapNode inNode in tempInputNodes)
                    {
                        foreach (MapNode adj in inNode.GetAdyacents())
                        {
                            if (originsNodes.ContainsKey(adj.origin))
                            {
                                // inNode.triangle.draw(Color.yellow);
                                // int index = originsNodes.IndexOf(adj.origin);
                                MapNode adjacent = originsNodes[adj.origin];

                                List<Arist> addArists = new List<Arist>();
                                List<Arist> deleteArists = new List<Arist>();

                                foreach (Arist inArist in inNode.adjacents.Values)
                                    foreach (Arist localArist in adjacent.adjacents.Values)
                                        if (inArist.origin == localArist.origin)
                                        {
                                            addArists.Add(localArist);
                                            deleteArists.Add(inArist);

                                            // PointNode.Static.DrawTwoPoints(localArist.p1, localArist.p2, Color.magenta);
                                            break;
                                        }

                                foreach (Arist aristAdd1 in addArists)
                                    foreach (Arist aristAdd2 in addArists)
                                        if (aristAdd1 != aristAdd2)
                                            foreach (PointNode point1 in aristAdd1.points)
                                                foreach (PointNode point2 in aristAdd2.points)
                                                {
                                                    /// Caso que una arista cierra un triangulo
                                                    point1.AddAdjacent(point2);
                                                    point2.AddAdjacent(point1);
                                                    point1.AddTriangle(inNode);
                                                    point2.AddTriangle(inNode);
                                                    PointNode.Static.DrawTwoPoints(point1.point, point2.point, Color.green);
                                                }

                                foreach (Arist aristOfInNode in inNode.adjacents.Values)
                                {
                                    if (deleteArists.Contains(aristOfInNode))
                                    {
                                        foreach (Arist aristOfInNode2 in inNode.adjacents.Values)
                                            foreach (PointNode pointInNode in aristOfInNode2.points)
                                                foreach (PointNode pointToDeleteAdj in aristOfInNode.points)
                                                {
                                                    pointInNode.RemoveAdjacent(pointToDeleteAdj);
                                                    pointToDeleteAdj.RemoveAdjacent(pointInNode);
                                                    // if (initMapNode == inNode)
                                                    //     if (initPointNode.adjacents.ContainsKey(pointToDeleteAdj))
                                                    //     {
                                                    //         initPointNode.RemoveAdjacent(pointToDeleteAdj);
                                                    //         // Point p = new Point(0.04f, 0, 0);
                                                    //         // PointNode.Static.DrawTwoPoints(initPointNode.point + p, pointToDeleteAdj.point + p, Color.red);
                                                    //     }
                                                    Point p = new Point(0.04f, 0, 0);
                                                    PointNode.Static.DrawTwoPoints(pointInNode.point + p, pointToDeleteAdj.point + p, Color.red);

                                                }
                                    }
                                    else
                                    {
                                        foreach (PointNode pointInNode in aristOfInNode.points)
                                            foreach (Arist localArist in addArists)
                                                foreach (PointNode pointInLocal in localArist.points)
                                                {
                                                    pointInNode.AddAdjacent(pointInLocal, inNode.MaterialCost(new Agent(1)));
                                                    pointInLocal.AddAdjacent(pointInNode, inNode.MaterialCost(new Agent(1)));
                                                    // if (inNode == initMapNode)
                                                    //     if (!initPointNode.adjacents.ContainsKey(pointInLocal))
                                                    //     {
                                                    //         initPointNode.AddAdjacent(pointInLocal, inNode.MaterialCost(new Agent(1)));
                                                    //         PointNode.Static.DrawTwoPoints(initPointNode.point, pointInLocal.point, Color.yellow);
                                                    //     }
                                                    PointNode.Static.DrawTwoPoints(pointInNode.point, pointInLocal.point, Color.yellow);
                                                    pointInLocal.AddTriangle(inNode);

                                                    // Debug.Log(pointInLocal.distance);
                                                }
                                    }
                                }
                            }
                        }
                    }
                    foreach (MapNode inNode in tempInputNodes)
                    {
                        originsNodes.Add(inNode.origin, inNode);
                    }
                }
                else
                {
                    Debug.Log("Entra al else");
                    Dictionary<MapNode, MapNode> originNew = new Dictionary<MapNode, MapNode>();
                    foreach (MapNode node in inNodes)
                        originNew.Add(node.origin, node);
                    origins.Add(triangle, originNew);
                }
            }
        }
    }
}