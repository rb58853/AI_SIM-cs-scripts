using System.Collections.Generic;
using Triangle_Map;
using BaseNode;
using DijkstraSpace;
using UnityEngine;
using Agent_Space;
using System;

namespace Point_Map
{
    public class PointNode : Node
    {
        public PointNode end { get; protected set; }
        public PointNode init { get; protected set; }
        public Point point { get; private set; }
        public Dictionary<PointNode, float> adjacents { get; private set; }
        public Arist arist { get; private set; }
        /// a point has maximum 2 triangles
        public List<MapNode> triangles { get; private set; }
        public Agent agent { get; private set; }
        public PointNode origin { get; private set; }
        public bool visitedInCreation = false;
        public Dictionary<Agent, bool> visitedInPath = new Dictionary<Agent, bool>();
        public bool visitedAsAdjacent = false;
        public bool inArist;

        //public bool borderPoint = false;

        public PointNode(Point point, PointNode end = null, bool inArist = true, Arist arist = null, Agent agent = null)
        {
            origin = this;
            this.agent = agent;
            this.arist = arist;
            this.inArist = inArist;
            this.point = point;
            adjacents = new Dictionary<PointNode, float>();
            distance = float.MaxValue;
            this.end = end;
            init = null;
            triangles = new List<MapNode>();
        }
        public void SetOrigin(PointNode point) { origin = point; }
        public float get_x() { return point.x; }
        public float get_y() { return point.y; }
        public float get_z() { return point.z; }

        public void AddTriangle(MapNode triangle) { triangles.Add(triangle); }
        public void SetEnd(PointNode node) { end = node; }
        public void SetInit(PointNode node) { init = node; }
        public void SetPoint(Point point) { this.point = point; }
        public void AddAdjacent(PointNode node, float value = 1) { adjacents.Add(node, value); }
        public void RemoveAdjacent(PointNode node) { adjacents.Remove(node); }
        public void RemoveAllAdjacents() { adjacents = new Dictionary<PointNode, float>(); }
        public float EuclideanDistance(PointNode node) { return point.Distance(node.point); }
        public override float Value()
        {
            if (init != null)
                return distance + EuclideanDistance(init) * init.adjacents[this];

            return distance;
        }
        public int CompareTo(PointNode other)
        {
            return this.value.CompareTo(other.value);
        }
        public override string ToString()
        {
            return point.ToString();
        }
        public override List<Node> GetAdyacents()
        {
            /// Esto en cuanto a eficiencia es malo, se esta sacrificando en eficiencia para ganar en genaricidad
            /// Cambiar el uso de lista por el uso de array en caso que deje usar el .toArray que no se cuanto cuesta tampoco
            List<Node> result = new List<Node>();
            foreach (Node adj in adjacents.Keys)
                result.Add(adj);
            return result;
        }
        public override float Distance(Node node)
        {
            return EuclideanDistance(node as PointNode) * adjacents[node as PointNode];
        }
        internal class Static
        {
            public static List<PointNode> CreatePointMap(PointNode initNode, Point end, Agent agent = null, float n = 1f, float cost = 1)
            {
                DateTime t0 = DateTime.Now;
                List<PointNode> result = new List<PointNode>();
                List<MapNode> mapNodes = agent.triangleList;

                // if (mapNodes.Count == 1)
                if (agent.initMapNodeCurrent.origin == agent.endMapNodeCurrent.origin)
                {
                    PointNode e = new PointNode(end, inArist: false);
                    // e.AddTriangle(mapNodes[mapNodes.Count - 1]);
                    e.AddTriangle(agent.initMapNodeCurrent);
                    PointNode i = initNode;

                    result.Add(i);

                    List<PointNode> temp = new List<PointNode>(); temp.Add(e);
                    CreateSimplePath(i, temp, agent, agent.currentNode, agent.currentNode.MaterialCost(agent), e, result);

                    result.Add(e);
                    return result;
                }

                PointNode endNode = new PointNode(end, inArist: false);
                endNode.AddTriangle(agent.initMapNodeCurrent);
                // endNode.AddTriangle(mapNodes[mapNodes.Count - 1]);

                initNode.visitedInCreation = true;
                result.Add(initNode);

                foreach (MapNode triangle in mapNodes)
                {
                    foreach (Arist arist in triangle.adjacents.Values)
                        // if (mapNodes.Contains(adj))
                        if (arist.points.Count == 0)
                            arist.ToPoints(n);

                    // if (triangle == mapNodes[0])
                    if (triangle == agent.endMapNodeCurrent)
                    {
                        triangle.triangle.draw(Color.blue);
                        foreach (MapNode adj in triangle.adjacents.Keys)
                        {
                            Arist arist = triangle.adjacents[adj];
                            // if (mapNodes.Contains(adj))
                            if (arist.points.Count > 0)
                                CreateSimplePath(initNode, arist.points, agent, triangle.origin,
                                          triangle.MaterialCost(agent), endNode, result);
                        }
                        // continue;
                    }

                    // if (triangle == mapNodes[mapNodes.Count - 1])
                    if (triangle == agent.initMapNodeCurrent)
                    {
                        triangle.triangle.draw(Color.cyan);

                        List<PointNode> temp = new List<PointNode>(); temp.Add(endNode);

                        foreach (MapNode adj in triangle.adjacents.Keys)
                        {
                            Arist arist = triangle.adjacents[adj];

                            // if (mapNodes.Contains(adj))
                            foreach (PointNode point in arist.points)
                                CreateSimplePath(point, temp, agent, triangle.origin,
                                triangle.MaterialCost(agent), endNode, result);
                        }
                        // continue;
                    }

                    foreach (MapNode adj1 in triangle.adjacents.Keys)
                        foreach (MapNode adj2 in triangle.adjacents.Keys)
                        {
                            Arist arist1 = triangle.adjacents[adj1];
                            Arist arist2 = triangle.adjacents[adj2];

                            // if (mapNodes.Contains(adj1) && mapNodes.Contains(adj2))
                            {
                                if (arist1 == arist2)
                                {
                                    if (!Agent_Space.Environment.moveInArist)
                                        continue;
                                    if (arist1.visited) continue;
                                    else arist1.visited = true;
                                }

                                foreach (PointNode p1 in arist1.points)
                                    CreateSimplePath(p1, arist2.points, agent, triangle.origin,
                                        triangle.MaterialCost(agent), endNode, result);
                            }
                        }
                }
                result.Add(endNode);
                return result;
            }
            static void CreateSimplePath(PointNode init, List<PointNode> list,
                Agent agent, MapNode mapNode, float cost, PointNode endNode,
                List<PointNode> result)
            {
                if (!init.visitedInCreation)
                {
                    init.visitedInCreation = true;
                    if (init != endNode)
                        result.Add(init);
                }

                foreach (PointNode node in list)
                {
                    try { init.AddAdjacent(node, cost); }
                    catch { Debug.Log("Error: Esta intentando agregar el adyacente ya existente " + node + " al punto " + init); }

                    if (Agent_Space.Environment.drawAllPossiblePaths)
                        DrawTwoPoints(init.point, node.point, Color.white);

                    if (!node.visitedInCreation)
                    {
                        node.visitedInCreation = true;
                        if (node != endNode)
                            result.Add(node);
                    }
                }
            }
            public static void DrawTwoPoints(Point p1, Point p2, Color color)
            {
                Vector3 a = new Vector3(p1.x, p1.y, p1.z);
                Vector3 b = new Vector3(p2.x, p2.y, p2.z);
                Debug.DrawLine(a, b, color, 50f);
            }
        }
    }
    public class PointPath
    {
        private Agent agent;
        public PointNode currentPoint { get; private set; }
        public PointNode nextPoint { get; private set; }
        public MapNode currentTriangle { get; private set; }
        public bool empty { get; private set; }
        int stopCount = 0;
        public bool stop { get => stopCount > 0; }
        Queue<PointNode> recentlyVisited;
        List<PointNode> extras = new List<PointNode>();

        public PointPath(Agent agent, MapNode triangle)
        {
            this.agent = agent;
            currentTriangle = triangle;
            empty = true;
            recentlyVisited = new Queue<PointNode>();
        }

        Heap q = new Heap();
        float tolerance = 0;
        public PointNode Pop(bool onCollision = false)
        {
            // if(currentPoint == nextPoint)

            if (onCollision)
            {
                if (currentPoint != null)
                {
                    if (Agent_Space.Environment.metaheuristic)
                    {
                        PointNode temp = new PointNode(agent.position);
                        foreach (PointNode adj in currentPoint.adjacents.Keys)
                            temp.AddAdjacent(adj, currentPoint.adjacents[adj]);
                        foreach (MapNode triangle in currentPoint.triangles)
                            temp.AddTriangle(triangle.origin);

                        if (!temp.triangles.Contains(currentTriangle.origin))
                            temp.AddTriangle(currentTriangle.origin);
                        temp.SetDistance(nextPoint.distance +
                         nextPoint.EuclideanDistance(temp) * currentTriangle.MaterialCost(agent));
                        currentPoint = temp;
                    }
                    else
                    {
                        currentPoint.SetPoint(agent.position);
                    }
                }
                else
                {
                    currentPoint = new PointNode(agent.position);
                    currentPoint.AddTriangle(currentTriangle);
                }
                // nextPoint = currentPoint;
            }
            else
                currentPoint = nextPoint;

            if (empty) return currentPoint;
            if (currentPoint.adjacents.Count == 0)
            {
                empty = true;
                CleanExtras();
                nextPoint = currentPoint;
                tolerance = Agent_Space.Environment.stopOnPathDistance;
                return currentPoint;
            }

            PushToRecently(currentPoint);

            q = new Heap();
            foreach (PointNode node in currentPoint.adjacents.Keys)
                if (!node.visitedInPath.ContainsKey(agent))
                {
                    if (node.distance == 0)
                    {
                        if (agent.endPointNode == node)
                        {
                            node.SetInit(currentPoint);
                            q.Push(node);
                        }
                        continue;
                    }
                    node.SetInit(currentPoint);
                    q.Push(node);
                }

            PointNode next = null;
            PointNode best = null;
            while (q.size > 0)
            {
                next = q.Pop() as PointNode;
                if (best == null)
                    best = next;

                if (next.distance >= float.MaxValue - 100)
                {
                    ///Esto es para que no llegue a un camino sin fin
                    currentPoint.RemoveAdjacent(next);
                    continue;
                }
                if (Agent_Space.Environment.stopOnPath)
                {
                    if (next.value > best.value + tolerance)
                    {
                        Stop();
                        nextPoint = currentPoint;
                        CleanExtras();
                        tolerance += 2;
                        return currentPoint;
                    }
                }

                MapNode triangleTemp = TriangleBetweenPoints(currentPoint, next);

                Tuple<bool, Agent> collision = null;

                if (!onCollision)
                    collision = Agent.Collision(currentPoint.point, next.point, agent, triangleTemp,
                    maxDistance: Agent_Space.Environment.distanceAnalizeCollision);
                else
                    collision = Agent.Collision(currentPoint.point, next.point, agent, triangleTemp,
                    maxDistance: Agent_Space.Environment.distanceAnalizeCollision);

                if (collision.Item1)
                {
                    next.SetFather(null);
                    Border(currentPoint, next, agent, triangleTemp,
                        triangleTemp.MaterialCost(agent), new List<Agent>(), q, next);
                }
                else
                {
                    nextPoint = next;
                    if (Agent_Space.Environment.drawPaths)
                        PointNode.Static.DrawTwoPoints(currentPoint.point, nextPoint.point, Color.cyan);

                    // GetCurrentTriangle();
                    currentTriangle = triangleTemp;
                    CleanExtras();
                    // Debug.Log(i + ") " + next.value);
                    tolerance = Agent_Space.Environment.stopOnPathDistance;
                    return next;
                }
            }

            Stop();
            CleanExtras();
            nextPoint = currentPoint;
            tolerance = Agent_Space.Environment.stopOnPathDistance;
            return currentPoint;
        }
        void CleanExtras()
        {
            foreach (PointNode extra in extras)
                (extra.father as PointNode).adjacents.Remove(extra);
            extras = new List<PointNode>();
        }
        void Stop()
        {
            if (stopCount <= 0)
                stopCount = Agent_Space.Environment.stopInCollision;
        }
        public void Move()
        {
            stopCount = 0;
        }
        public void EmptyMove()
        {
            stopCount--;
        }

        MapNode GetCurrentTriangle()
        {
            foreach (MapNode node1 in currentPoint.triangles)
                foreach (MapNode node2 in nextPoint.triangles)
                    if (node1.triangle == node2.triangle)
                    { currentTriangle = node1.origin; return currentTriangle; }
            Debug.Log("No encontro triangulo comun");
            return currentTriangle;
        }
        public void SetCurrentTriangle(MapNode triangle)
        {
            currentTriangle = triangle;
        }
        MapNode TriangleBetweenPoints(PointNode point1, PointNode point2)
        {
            // Debug.Log(point1 +"(p1) tiene cantidad de triangulos = " + point1.triangles.Count + "     "+point2 +"(p2) tiene cantidad de triangulos = " + point2.triangles.Count);

            foreach (MapNode node1 in point1.triangles)
                foreach (MapNode node2 in point2.triangles)
                    if (node1.origin == node2.origin)
                        return node1.origin;

            Debug.Log("Los puntos " + point1 + " y " + point2 + "No tienen triangulo comun");
            foreach (MapNode t in point1.triangles)
            {
                Debug.Log("P1 triangulos = " + t);
            }
            foreach (MapNode t in point2.triangles)
            {
                Debug.Log("P2 triangulos = " + t);
            }
            return null;
        }

        public void pushMetaMap(PointNode currentPosition)
        {
            stopCount = 0;
            recentlyVisited = new Queue<PointNode>();
            empty = false;
            this.nextPoint = currentPosition;
            currentPoint = null;
            Debug.Log("Selecciono camino de meta");
        }
        public void PushPointMap(PointNode[] nodes)
        {
            stopCount = 0;

            recentlyVisited = new Queue<PointNode>();

            empty = false;
            ///Invertir la direccion de las aristas ultimo y primero
            PointNode node = nodes[0];

            List<PointNode> temp = new List<PointNode>();
            foreach (PointNode adj in node.adjacents.Keys) temp.Add(adj);

            foreach (PointNode adj in temp)
            {
                adj.AddAdjacent(node, node.adjacents[adj]);
                node.RemoveAdjacent(adj);
            }

            foreach (PointNode point in nodes)
                if (point.adjacents.ContainsKey(nodes[nodes.Length - 1]))
                {
                    nodes[nodes.Length - 1].AddAdjacent(point, point.adjacents[nodes[nodes.Length - 1]]);
                    point.RemoveAdjacent(nodes[nodes.Length - 1]);
                }

            currentPoint = null;
            nextPoint = nodes[nodes.Length - 1];
        }
        public void PushPointMap(PointNode initNode, PointNode endNode, MapNode endMapNode)
        {
            stopCount = 0;
            recentlyVisited = new Queue<PointNode>();
            empty = false;

            ///Invertir la direccion de las aristas ultimo y primero
            PointNode node = initNode;

            List<PointNode> temp = new List<PointNode>();
            foreach (PointNode adj in node.adjacents.Keys) temp.Add(adj);

            foreach (PointNode adj in temp)
            {
                adj.AddAdjacent(node, node.adjacents[adj]);
                // PointNode.Static.DrawTwoPoints(node.point, adj.point, Color.green);
                node.RemoveAdjacent(adj);
            }

            foreach (Arist arist in endMapNode.adjacents.Values)
                foreach (PointNode point in arist.points)
                    if (point.adjacents.ContainsKey(endNode))
                    {
                        // PointNode.Static.DrawTwoPoints(endNode.point, point.point, Color.yellow);
                        endNode.AddAdjacent(point, point.adjacents[endNode]);
                        point.RemoveAdjacent(endNode);
                    }

            currentPoint = null;
            nextPoint = endNode;
        }
        public void PushCurrenTriangle(MapNode triangle)
        {
            currentTriangle = triangle;
        }
        void PushToRecently(PointNode node)
        {
            if (recentlyVisited.Count >= 20)
                recentlyVisited.Dequeue().visitedInPath.Remove(agent);
            if (!currentPoint.visitedInPath.ContainsKey(agent))
                node.visitedInPath.Add(agent, true);
            recentlyVisited.Enqueue(node);
        }
        public void clear()
        {
            while (recentlyVisited.Count > 0)
                recentlyVisited.Dequeue().visitedInPath.Remove(agent);
            CleanExtras();
        }
        void Border(PointNode initIn, PointNode end,
                Agent agent, MapNode mapNode, float cost,
                List<Agent> visitedObstacles, Heap q, PointNode destination)
        {
            if (!Agent_Space.Environment.createBorder) return;

            PointNode init = new PointNode(initIn.point, inArist: false);
            init.AddTriangle(mapNode);
            initIn.AddAdjacent(init);
            init.SetFather(initIn);
            CreateObstacleBorder(init, end, agent, mapNode, cost, visitedObstacles, destination);

            if (init.adjacents.Count == 0)//no se encontro nada que no sea lo mismo 
            {
                initIn.RemoveAdjacent(init);
                return;
            }
            extras.Add(init);
            q.Push(init);
        }
        void CreateObstacleBorder(PointNode init, PointNode end,
                Agent agent, MapNode mapNode, float cost,
                List<Agent> visitedObstacles, PointNode destination)
        {
            Tuple<bool, Agent> collision = Agent.Collision(init.point, end.point, agent, mapNode, 1.0f);
            if (collision.Item1)
            {
                if (!visitedObstacles.Contains(collision.Item2))
                {
                    if ((int)Agent_Space.Environment.qualityBorder < 2)
                        if (destination.father != null)
                            return;

                    visitedObstacles.Add(collision.Item2);

                    Point center = collision.Item2.position;

                    PointNode W = new PointNode(center -
                        Point.VectorUnit(init.point, center)
                        * (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false); W.AddTriangle(mapNode);
                    ///West view from lefth to rigth

                    PointNode e = new PointNode(center +
                        Point.VectorUnit(init.point, center) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false); e.AddTriangle(mapNode);
                    ///Est view lefth to rigth

                    PointNode sw = new PointNode(W.point +
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false); sw.AddTriangle(mapNode);
                    ///SouthWest view lefth to rigth

                    PointNode s = new PointNode(center +
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false); s.AddTriangle(mapNode);
                    ///South view lefth to rigth

                    PointNode se = new PointNode(e.point +
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false); se.AddTriangle(mapNode);
                    ///South view lefth to rigth

                    PointNode nw = new PointNode(W.point -
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false); nw.AddTriangle(mapNode);
                    ///SouthWest view lefth to rigth

                    PointNode n = new PointNode(center -
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false); n.AddTriangle(mapNode);
                    ///South view lefth to rigth

                    PointNode ne = new PointNode(e.point -
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false); ne.AddTriangle(mapNode);
                    ///South view lefth to rigth

                    Queue<PointNode> down = new Queue<PointNode>();
                    down.Enqueue(nw); /*down.Enqueue(n);*/ down.Enqueue(ne); down.Enqueue(e); down.Enqueue(destination);
                    Queue<PointNode> up = new Queue<PointNode>();
                    up.Enqueue(sw); /*up.Enqueue(s);*/ up.Enqueue(se); up.Enqueue(e); up.Enqueue(destination);


                    // init.visitedInCreation = true;
                    PointNode node1 = init;

                    while (down.Count > 0)
                    {
                        if (!node1.visitedInCreation) break;
                        if (!Agent.Collision(node1.point, destination.point, agent, mapNode, 1.0f).Item1)
                        {
                            if (!node1.adjacents.ContainsKey(destination))
                                node1.AddAdjacent(destination, cost);

                            destination.SetFather(node1);
                            destination.SetInit(node1);
                            setDistances(destination, cost);
                            if (Agent_Space.Environment.drawBorder)
                                PointNode.Static.DrawTwoPoints(node1.point, destination.point, Color.red);

                            break;
                        }

                        PointNode node2 = down.Dequeue();

                        if (Agent_Space.Environment.dinamycPointToAllArists)
                            setAdjacentsFromAristTriangle(node2, mapNode, agent);

                        bool c1 = false;
                        bool c2 = false;
                        if (Agent_Space.Environment.onlyTriangleBorder)
                        {
                            c1 = mapNode.triangle.PointIn(node2.point);
                            c2 = !Agent.Collision(node1.point, node2.point, agent, mapNode, 1.0f).Item1;
                        }
                        else
                        {
                            c1 = node2.point.InTriangles(collision.Item2.ocupedTriangles);
                            c2 = !Agent.Collision(node1.point, node2.point, agent, collision.Item2.ocupedNodes, 1.0f).Item1;
                        }

                        if (c1)
                            if (c2)
                            {
                                node1.AddAdjacent(node2, cost);
                                node2.SetFather(node1);
                                node2.SetInit(node1);
                                node2.visitedInCreation = true;

                                // if (node2 == destination)
                                // {
                                //     setDistances(destination, cost);
                                //     if ((int)Agent_Space.Environment.qualityBorder <= 1)
                                //         return;
                                //     break;
                                // }

                                if (Agent_Space.Environment.drawBorder)
                                    PointNode.Static.DrawTwoPoints(node1.point, node2.point, Color.red);

                                if ((int)Agent_Space.Environment.qualityBorder >= 1)
                                    CreateObstacleBorder(node2, destination, agent, mapNode, cost, visitedObstacles, destination);
                            }
                            else
                                CreateObstacleBorder(node1, node2, agent, mapNode, cost, visitedObstacles, destination);

                        node1 = node2;
                    }

                    node1 = init;
                    while (up.Count > 0)
                    {
                        if (!node1.visitedInCreation) break;
                        if (!Agent.Collision(node1.point, destination.point, agent, mapNode, 1.0f).Item1)
                        {
                            if (!node1.adjacents.ContainsKey(destination))
                                node1.AddAdjacent(destination, cost);

                            destination.SetFather(node1);
                            destination.SetInit(node1);
                            setDistances(destination, cost);

                            if (Agent_Space.Environment.drawBorder)
                                PointNode.Static.DrawTwoPoints(node1.point, destination.point, Color.red);

                            break;
                        }

                        PointNode node2 = up.Dequeue();

                        if (Agent_Space.Environment.dinamycPointToAllArists)
                            setAdjacentsFromAristTriangle(node2, mapNode, agent);

                        bool c1 = false;
                        bool c2 = false;
                        if (Agent_Space.Environment.onlyTriangleBorder)
                        {
                            c1 = mapNode.triangle.PointIn(node2.point);
                            c2 = !Agent.Collision(node1.point, node2.point, agent, mapNode, 1.0f).Item1;
                        }
                        else
                        {
                            c1 = node2.point.InTriangles(collision.Item2.ocupedTriangles);
                            c2 = !Agent.Collision(node1.point, node2.point, agent, collision.Item2.ocupedNodes, 1.0f).Item1;
                        }

                        if (c1)
                            if (c2)
                            {
                                node1.AddAdjacent(node2, cost);
                                node2.SetInit(node1);
                                node2.SetFather(node1);
                                node2.visitedInCreation = true;

                                // if (node2 == destination)
                                // {
                                //     setDistances(destination, cost);
                                //     if ((int)Agent_Space.Environment.qualityBorder <= 1)
                                //         return;
                                //     break;
                                // }

                                if (Agent_Space.Environment.drawBorder)
                                    PointNode.Static.DrawTwoPoints(node1.point, node2.point, Color.red);

                                if ((int)Agent_Space.Environment.qualityBorder >= 1)
                                    CreateObstacleBorder(node2, destination, agent, mapNode, cost, visitedObstacles, destination);
                            }
                            else
                                CreateObstacleBorder(node1, node2, agent, mapNode, cost, visitedObstacles, destination);

                        node1 = node2;
                    }
                }
            }
        }
        void setDistances(PointNode destination, float cost)
        {
            if (destination.father != null)
            {
                PointNode temp = destination;

                while (temp.father.distance >= float.MaxValue - 100)
                {
                    float min = Math.Min(temp.distance + temp.EuclideanDistance(temp.father as PointNode) * cost,
                     temp.father.distance);

                    temp.father.SetDistance(min);
                    temp = temp.father as PointNode;
                    if (temp.father == null) break;
                }
            }
        }
        void setAdjacentsFromAristTriangle(PointNode node, MapNode mapNode, Agent agent)
        {
            foreach (Arist arist in mapNode.adjacents.Values)
            {
                List<PointNode> points = arist.points;
                if (points.Count == 0 || points[0].distance >= float.MaxValue - 100)
                    continue;
                else
                    foreach (PointNode point in points)
                    {
                        node.AddAdjacent(point, mapNode.MaterialCost(agent));
                        if (node.distance > point.distance + point.Distance(node))
                            node.SetDistance(point.distance + point.Distance(node));
                    }
            }
        }
    }
}