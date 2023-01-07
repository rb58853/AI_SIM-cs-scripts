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

        /// a point has maximum 2 triangles
        public List<MapNode> triangles { get; private set; }
        public bool visitedInCreation = false;
        public bool visitedAsAdjacent = false;
        public bool inArist { get; private set; }

        //public bool borderPoint = false;

        public PointNode(Point point, PointNode end = null, bool inArist = true)
        {
            this.inArist = inArist;
            this.point = point;
            adjacents = new Dictionary<PointNode, float>();
            distance = float.MaxValue;
            this.end = end;
            init = null;
            triangles = new List<MapNode>();
        }
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
        //float Heuristic(PointNode endNode) { return EuclideanDistance(endNode); }
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

                if (mapNodes.Count == 1)
                {
                    PointNode e = new PointNode(end, inArist: false);
                    PointNode i = initNode;
                    e.SetEnd(e);
                    result.Add(i);
                    i.visitedInCreation = true;

                    List<PointNode> temp = new List<PointNode>(); temp.Add(e);
                    CreateSimplePath(i, temp, agent, agent.currentNode, agent.currentNode.MaterialCost(agent), e, result);

                    result.Add(e);
                    return result;
                }

                PointNode endNode = new PointNode(end, inArist: false); endNode.SetEnd(endNode);
                initNode.visitedInCreation = true;

                //result.Add(initNode);

                foreach (MapNode triangle in mapNodes)
                {
                    ///Lo proximo se va creando dinamicamente cuando se va creando el camino de triangulos.
                    ///Tambien se agrega a un mapa de puntos del agente estos dinamicamente mientras se crean los 
                    ///triangulos

                    //foreach (MapNode adj in triangle.adjacents.Keys)
                    //    if (mapNodes.Contains(adj))
                    //        if (triangle.adjacents[adj].points.Count == 0)
                    //            triangle.adjacents[adj].ToPoints(n);

                    if (triangle == mapNodes[0])
                    {
                        foreach (MapNode adj in triangle.adjacents.Keys)
                        {
                            Arist arist = triangle.adjacents[adj];

                            if (mapNodes.Contains(adj))
                            {
                                CreateSimplePath(initNode, arist.points, agent, triangle.origin,
                                          triangle.MaterialCost(agent), endNode, result);
                                Debug.Log("La arista tiene cantidad de puntos = " + arist.points.Count);
                            }
                        }
                        continue;
                    }

                    if (triangle == mapNodes[mapNodes.Count - 1])
                    {
                        List<PointNode> temp = new List<PointNode>(); temp.Add(endNode);

                        foreach (MapNode adj in triangle.adjacents.Keys)
                        {
                            Arist arist = triangle.adjacents[adj];
                            if (mapNodes.Contains(adj))
                                foreach (PointNode point in arist.points)
                                    CreateSimplePath(point, temp, agent, triangle.origin,
                                        triangle.MaterialCost(agent), endNode, result);
                        }
                        continue;
                    }

                    foreach (MapNode adj1 in triangle.adjacents.Keys)
                        foreach (MapNode adj2 in triangle.adjacents.Keys)
                        {
                            Arist arist1 = triangle.adjacents[adj1];
                            Arist arist2 = triangle.adjacents[adj2];

                            if (arist1 != arist2 &&
                                mapNodes.Contains(adj1) &&
                                mapNodes.Contains(adj2))
                                foreach (PointNode p1 in arist1.points)
                                    CreateSimplePath(p1, arist2.points, agent, triangle.origin,
                                        triangle.MaterialCost(agent), endNode, result);
                        }
                }

                result.Add(endNode);
                Debug.Log("Crear el mapa de puntos demoro: " + (DateTime.Now - t0));
                return result;

                //Debug.Log("Count del resultado del mapa de puntos " + agent.pointsMap.Count);
                //agent.pointsMap.Add(endNode);
                //return agent.pointsMap;
            }

            static void CreateSimplePath(PointNode init, List<PointNode> list,
                Agent agent, MapNode mapNode, float cost, PointNode endNode,
                List<PointNode> result)
            {

                //if (!init.visitedInCreation) return;

                List<PointNode> endList = new List<PointNode>();
                endList.Add(init);

                if (!result.Contains(init)) /// Esto no me gusta en cuanto a eficiencia FFF
                    result.Add(init);

                init.SetDistance(0);
                Heap q = new Heap(init);


                List<Agent> visitedObstacles = new List<Agent>();

                List<PointNode> temp = new List<PointNode>();
                foreach (PointNode node in list)
                    temp.Add(node);

                while (temp.Count > 0 && q.size > 0)
                {
                    PointNode current = q.Pop() as PointNode;

                    for (int i = 0; i < temp.Count; i++)
                    {
                        PointNode end = temp[i];

                        if (Agent_Space.Environment.pathWithAgents)
                        {
                            Tuple<bool, Agent> collision = Agent.Collision(current.point, end.point, agent, mapNode, 1.0f);

                            if (collision.Item1)
                            {
                                CreateObstacleBorder(current, end, agent, mapNode, cost, result,
                                    visitedObstacles, q, endList, endNode, end);
                                current.AddAdjacent(end, float.MaxValue - 10000 + cost);
                                continue;
                            }
                        }

                        current.AddAdjacent(end, cost);

                        DrawTwoPoints(current.point, end.point, Color.yellow);

                        end.visitedInCreation = true;
                        //if (!result.Contains(end)) /// Esto no me gusta en cuanto a eficiencia FFF
                        //    result.Add(end);
                        temp.Remove(end); i--;
                    }
                    foreach (PointNode node in endList)
                    {
                        ///Resetear la distancia en todos los nodos que me interesan, solo me interesa los init y los nodos extras que se agregaron
                        ///xq los end son init en otra entrada.
                        node.SetDistance(float.MaxValue);
                    }
                    ///Unico end que no sera nunk un init, si funciona bien se pone arriba
                    endNode.SetDistance(float.MaxValue);
                }

            }

            static void CreateObstacleBorder(PointNode init, PointNode end,
                Agent agent, MapNode mapNode, float cost, List<PointNode> result,
                List<Agent> visitedObstacles, Heap q, List<PointNode> endList,
                PointNode endNode, PointNode destination)
            {
                ///De alguna manera hay que hacer que esto pueda volver a visitar otros agentes para detectar colisiones 
                ///para poder hacer el bordeo mejor

                Tuple<bool, Agent> collision = Agent.Collision(init.point, end.point, agent, mapNode, 1.0f);
                if (collision.Item1)
                {
                    //DrawTwoPoints(init.point, collision.Item2.position, Color.black);

                    if (!visitedObstacles.Contains(collision.Item2))
                    {
                        visitedObstacles.Add(collision.Item2);

                        Point center = collision.Item2.position;

                        PointNode W = new PointNode(center -
                            Point.VectorUnit(init.point, center)
                            * (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder,
                            endNode, inArist: false);///West view from lefth to rigth

                        PointNode e = new PointNode(center +
                            Point.VectorUnit(init.point, center) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode, inArist: false);///Est view lefth to rigth

                        PointNode sw = new PointNode(W.point +
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode, inArist: false);///SouthWest view lefth to rigth

                        PointNode s = new PointNode(center +
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode, inArist: false);///South view lefth to rigth

                        PointNode se = new PointNode(e.point +
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode, inArist: false);///South view lefth to rigth

                        PointNode nw = new PointNode(W.point -
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode, inArist: false);///SouthWest view lefth to rigth

                        PointNode n = new PointNode(center -
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode, inArist: false);///South view lefth to rigth

                        PointNode ne = new PointNode(e.point -
                            Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                            (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                            , endNode, inArist: false);///South view lefth to rigth


                        //DrawTwoPoints(W.point, nw.point, Color.red);
                        //DrawTwoPoints(nw.point, n.point, Color.red);
                        //DrawTwoPoints(n.point, ne.point, Color.red);
                        //DrawTwoPoints(ne.point, ne.point, Color.red);



                        Queue<PointNode> up = new Queue<PointNode>();
                        /*up.Enqueue(W);*/
                        up.Enqueue(nw); /*up.Enqueue(n);*/ up.Enqueue(ne); up.Enqueue(e);
                        Queue<PointNode> down = new Queue<PointNode>();
                        down.Enqueue(sw); /*down.Enqueue(s);*/ down.Enqueue(se); down.Enqueue(e);


                        PointNode node1 = init;

                        while (up.Count > 0)
                        {
                            if (!node1.visitedInCreation) break;

                            if (!Agent.Collision(node1.point, destination.point, agent, mapNode, 1.0f).Item1)
                            {
                                //node1.AddAdjacent(destination, cost);

                                //DrawTwoPoints(node1.point, destination.point, Color.green);

                                //destination.visitedInCreation = true;
                                //destination.SetDistance(node1.distance + node1.EuclideanDistance(destination));
                                ////q.Push(destination);
                                //endList.Add(destination);
                                //break;
                            }

                            PointNode node2 = up.Dequeue();

                            if (mapNode.triangle.PointIn(node2.point))
                                if (!Agent.Collision(node1.point, node2.point, agent, mapNode, 1.0f).Item1)
                                {
                                    node1.AddAdjacent(node2, cost);

                                    DrawTwoPoints(node1.point, node2.point, Color.green);

                                    node2.visitedInCreation = true;
                                    node2.SetDistance(node1.distance + node1.EuclideanDistance(node2));
                                    q.Push(node2);
                                    result.Add(node2);
                                    endList.Add(node2);
                                }
                                else
                                {
                                    CreateObstacleBorder(node1, node2, agent, mapNode, cost, result, visitedObstacles, q, endList, endNode, destination);
                                }

                            node1 = node2;
                        }

                        node1 = W;
                        while (down.Count > 0)
                        {
                            if (!node1.visitedInCreation) break;

                            if (!Agent.Collision(node1.point, destination.point, agent, mapNode, 1.0f).Item1)
                            {
                                //node1.AddAdjacent(destination, cost);

                                //DrawTwoPoints(node1.point, destination.point, Color.green);

                                //destination.visitedInCreation = true;
                                //destination.SetDistance(node1.distance + node1.EuclideanDistance(destination));
                                ////q.Push(destination);
                                //endList.Add(destination);
                                //break;
                            }


                            PointNode node2 = down.Dequeue();
                            if (mapNode.triangle.PointIn(node2.point))
                                if (!Agent.Collision(node1.point, node2.point, agent, mapNode, 1.0f).Item1)
                                {
                                    node1.AddAdjacent(node2, cost);

                                    DrawTwoPoints(node1.point, node2.point, Color.green);

                                    node2.visitedInCreation = true;
                                    node2.SetDistance(node1.distance + node1.EuclideanDistance(node2));
                                    q.Push(node2);
                                    result.Add(node2);
                                    endList.Add(node2);
                                }
                                else
                                {
                                    CreateObstacleBorder(node1, node2, agent, mapNode, cost, result, visitedObstacles, q, endList, endNode, destination);
                                }

                            node1 = node2;
                        }
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
        public MapNode currentNode { get; private set; }
        public bool empty { get; private set; }


        public PointPath(Agent agent)
        {
            this.agent = agent;
            currentNode = null;
            empty = true;
        }
        public PointNode Pop(bool onCollision = false)
        {
            if (empty) return nextPoint;

            if (onCollision)
                currentPoint.SetPoint(agent.position);
            else
            {
                if (nextPoint.inArist)
                    foreach (MapNode triangle in nextPoint.triangles)
                        if (triangle != currentNode)
                        {
                            currentNode = triangle.origin;
                            break;
                        }
                currentPoint = nextPoint;
            }

            Heap q = new Heap();
            foreach (PointNode node in currentPoint.adjacents.Keys)
            {
                node.SetInit(currentPoint);
                q.Push(node);
            }
            Debug.Log("Cantidad de elementos en el heap pa caminar: " + q.size);

            PointNode next = null;

            while (q.size > 0)
            {
                next = q.Pop() as PointNode;

                Tuple<bool, Agent> collision = Agent.Collision(agent.position, next.point, agent, currentNode);
                if (collision.Item1)
                {
                    next.SetFather(null);
                    Border(currentPoint, next, agent, currentNode,
                        currentNode.MaterialCost(agent), new List<Agent>(), q, next);
                    currentPoint.RemoveAdjacent(next);
                }
                else
                {
                    nextPoint = next;
                    if (next.adjacents.Count == 0) empty = true;
                    return next;
                }
            }

            empty = true;
            return currentPoint;
        }
        public void PushPointMap(PointNode[] nodes)
        {
            empty = false;
            ///Invertir la direccion de las aristas
            foreach (PointNode node in nodes)
                foreach (PointNode adj in node.adjacents.Keys)
                {
                    adj.AddAdjacent(node, node.adjacents[adj]);
                    node.RemoveAdjacent(adj);
                }
            currentPoint = null;
            nextPoint = nodes[nodes.Length - 1];
        }
        public void PushPointMap(List<PointNode> nodes)
        {
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
                if (point.adjacents.ContainsKey(nodes[nodes.Count - 1]))
                {
                    nodes[nodes.Count - 1].AddAdjacent(point, point.adjacents[nodes[nodes.Count - 1]]);
                    point.RemoveAdjacent(nodes[nodes.Count - 1]);
                }

            currentPoint = null;
            nextPoint = nodes[nodes.Count - 1];
        }

        public void PushCurrenTriangle(MapNode triangle)
        {
            currentNode = triangle;
        }

        public void clear()
        {
            currentNode = null;
            nextPoint = null;
            currentPoint = null;
        }
        void Border(PointNode initIn, PointNode end,
                Agent agent, MapNode mapNode, float cost,
                List<Agent> visitedObstacles, Heap q, PointNode destination)
        {
            PointNode init = new PointNode(initIn.point, inArist: false);
            initIn.AddAdjacent(init);
            CreateObstacleBorder(init, end, agent, mapNode, cost, visitedObstacles, destination);

            if (destination.father != null)/// Si se llego al destino por aqui, push
                q.Push(init);
        }
        void CreateObstacleBorder(PointNode init, PointNode end,
                Agent agent, MapNode mapNode, float cost,
                List<Agent> visitedObstacles, PointNode destination)
        {
            ///De alguna manera hay que hacer que esto pueda volver a visitar otros agentes para detectar colisiones 
            ///para poder hacer el bordeo mejor

            Tuple<bool, Agent> collision = Agent.Collision(init.point, end.point, agent, mapNode, 1.0f);
            if (collision.Item1)
            {
                //DrawTwoPoints(init.point, collision.Item2.position, Color.black);

                if (!visitedObstacles.Contains(collision.Item2))
                {
                    visitedObstacles.Add(collision.Item2);

                    Point center = collision.Item2.position;

                    PointNode W = new PointNode(center -
                        Point.VectorUnit(init.point, center)
                        * (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false);
                    ///West view from lefth to rigth

                    PointNode e = new PointNode(center +
                        Point.VectorUnit(init.point, center) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false);
                    ///Est view lefth to rigth

                    PointNode sw = new PointNode(W.point +
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false);
                    ///SouthWest view lefth to rigth

                    PointNode s = new PointNode(center +
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false);
                    ///South view lefth to rigth

                    PointNode se = new PointNode(e.point +
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false);
                    ///South view lefth to rigth

                    PointNode nw = new PointNode(W.point -
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false);
                    ///SouthWest view lefth to rigth

                    PointNode n = new PointNode(center -
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false);
                    ///South view lefth to rigth

                    PointNode ne = new PointNode(e.point -
                        Point.VectorUnit(Point.OrtogonalVector(init.point, W.point)) *
                        (agent.radius + collision.Item2.radius) * Agent_Space.Environment.collisionBorder
                        , inArist: false);
                    ///South view lefth to rigth

                    Queue<PointNode> up = new Queue<PointNode>();
                    up.Enqueue(nw); /*up.Enqueue(n);*/ up.Enqueue(ne); up.Enqueue(e);
                    Queue<PointNode> down = new Queue<PointNode>();
                    down.Enqueue(sw); /*down.Enqueue(s);*/ down.Enqueue(se); down.Enqueue(e);


                    init.visitedInCreation = true;
                    PointNode node1 = init;

                    while (up.Count > 0)
                    {

                        if (!node1.visitedInCreation) break;
                        if (!Agent.Collision(node1.point, destination.point, agent, mapNode, 1.0f).Item1)
                        {
                            node1.AddAdjacent(destination, cost);
                            destination.SetFather(node1);
                            destination.SetInit(node1);
                            setDistances(destination, cost);
                            break;
                        }

                        PointNode node2 = up.Dequeue();

                        //if (mapNode.triangle.PointIn(node2.point))
                        if (!Agent.Collision(node1.point, node2.point, agent, mapNode, 1.0f).Item1)
                        {
                            node1.AddAdjacent(node2, cost);
                            node2.SetFather(node1);
                            node2.SetInit(node1);
                            node2.visitedInCreation = true;

                            PointNode.Static.DrawTwoPoints(node1.point, node2.point, Color.green);
                        }
                        else
                            CreateObstacleBorder(node1, node2, agent, mapNode, cost, visitedObstacles, destination);

                        node1 = node2;
                    }

                    node1 = init;
                    while (down.Count > 0)
                    {
                        if (!node1.visitedInCreation) break;
                        if (!Agent.Collision(node1.point, destination.point, agent, mapNode, 1.0f).Item1)
                        {
                            node1.AddAdjacent(destination, cost);
                            destination.SetFather(node1);
                            destination.SetInit(node1);
                            setDistances(destination, cost);
                            break;
                        }

                        PointNode node2 = down.Dequeue();

                        //if (mapNode.triangle.PointIn(node2.point))
                        if (!Agent.Collision(node1.point, node2.point, agent, mapNode, 1.0f).Item1)
                        {
                            node1.AddAdjacent(node2, cost);
                            node2.SetInit(node1);
                            node2.SetFather(node1);
                            node2.visitedInCreation = true;

                            PointNode.Static.DrawTwoPoints(node1.point, node2.point, Color.green);
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
                    temp.father.SetDistance(temp.distance + temp.EuclideanDistance(temp.father as PointNode) * cost);
                    temp = temp.father as PointNode;
                    if (temp.father == null) break;
                }
            }
        }
    }
}