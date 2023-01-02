using BaseNode;

namespace Agent_Space
{
    public enum Material
    {
        basic = 10,
        fire = 1,
        water = 5
    }
    class Environment
    {
        public readonly static Map map = new Map();

        /// <summary> more value => best path  |  less value => best code eficience </summary>
        public readonly static float densityPath = 0.5f;
        /// <summary> count of nodes to visit(with bfs) after found the destination node </summary>
        public readonly static int bfsArea = 40;
        /// <summary> heuristic weigth for path between triangles </summary>
        public readonly static float heuristicTriangleWeigth = 2f;
        /// <summary> heuristic weigth for path between points </summary>
        public readonly static float heuristicPointWeigth = 1f;
        /// <summary> width for border collision</summary>
        public readonly static float collisionBorder = 1.3f;
    }
    class Obsolete
    {
        //public void SetOcupedFromPosition()
        //{
        //    Queue<MapNode> ocuped = new Queue<MapNode>();

        //    foreach (MapNode node in ocupedNodes)
        //        if (node != currentNode)
        //            node.RemoveAgent(this);

        //    ocuped.Enqueue(currentNode);
        //    currentNode.AddAgent(this);
        //    ocupedNodes = new List<MapNode>();
        //    ocupedNodes.Add(currentNode);

        //    int o = 0;
        //    while (ocuped.Count > 0)
        //    {
        //        o++;
        //        Debug.Log("over = " + o);
        //        MapNode node = ocuped.Dequeue();

        //        foreach (MapNode adj in node.adjacents.Keys)
        //            if (position.DistanceToTriangle(adj.triangle) < radius)
        //                if (!ocupedNodes.Contains(adj))
        //                {
        //                    ocuped.Enqueue(adj);
        //                    adj.AddAgent(this);
        //                    ocupedNodes.Add(adj);
        //                }
        //    }
        //    Debug.Log(name + " tiene ocupados triangulos = " + ocupedNodes.Count);
        //}
        #region Mas preciso pero mucho mas costoso que el actual, metodo de la clase PointMap.Static
        //static void CreateSimplePath(PointNode init, List<PointNode> list,
        //      Agent agent, MapNode mapNode, float cost, PointNode endNode,
        //      List<PointNode> result, List<Agent> visitedObstacles)
        //{

        //    if (!init.visitedInCreation) return;
        //    PointNode before = null;
        //    int index = 0;

        //    result.Add(init);
        //    foreach (PointNode end in list)
        //    {
        //        Tuple<bool, Agent> collision = Collision(init, end, agent, mapNode);
        //        if (collision.Item1)
        //        {
        //            if (visitedObstacles.Contains(collision.Item2))
        //                continue;

        //            PointNode another1 = null;
        //            PointNode another2 = null;
        //            //if (before != null && !visitedObstacles.Contains(collision.Item2))
        //            //  another = new PointNode(IntersectedOrtogonalVectors(init.point, before.point, collision.Item2.position), endNode);
        //            //else
        //            another1 = new PointNode(GeneratedPoint(init.point, agent, collision.Item2), endNode);
        //            another2 = new PointNode(GeneratedPoint(init.point, agent, collision.Item2, negative: true), endNode);
        //            //Debug.Log(another);

        //            if (mapNode.triangle.PointIn(another1.point))
        //                if (!Collision(init, another1, agent, mapNode).Item1)
        //                {
        //                    init.AddAdjacent(another1, cost);
        //                    result.Add(another1);
        //                    another1.visitedInCreation = true;
        //                    visitedObstacles.Add(collision.Item2);
        //                    //CreateSimplePath(another1, list.GetRange(index, list.Count - index), agent, mapNode, cost, endNode, result, visitedObstacles);
        //                    CreateSimplePath(another1, list, agent, mapNode, cost, endNode, result, visitedObstacles);
        //                    //visitedObstacles.Remove(collision.Item2); ///More path, but very much complex
        //                }

        //            if (mapNode.triangle.PointIn(another2.point))
        //                if (!Collision(init, another2, agent, mapNode).Item1)
        //                {
        //                    init.AddAdjacent(another2, cost);
        //                    result.Add(another2);
        //                    another2.visitedInCreation = true;
        //                    visitedObstacles.Add(collision.Item2);
        //                    CreateSimplePath(another2, list, agent, mapNode, cost, endNode, result, visitedObstacles);
        //                    //CreateSimplePath(another2, list.GetRange(index, list.Count - index), agent, mapNode, cost, endNode, result, visitedObstacles);
        //                    //visitedObstacles.Remove(collision.Item2); ///More path, but very much complex
        //                }

        //            continue;
        //        }

        //        Vector3 a = new Vector3(init.point.x, init.point.y, init.point.z);
        //        Vector3 b = new Vector3(end.point.x, end.point.y, end.point.z);
        //        Debug.DrawLine(a, b, Color.black, 50f);

        //        init.AddAdjacent(end, cost);
        //        end.visitedInCreation = true;
        //        before = end;

        //        index++;

        //    }
        //}
        #endregion
    }
}
