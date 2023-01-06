using BaseNode;

namespace Agent_Space
{
    public enum Material
    {
        basic = 10,
        fire = 80,
        water = 30,
        hierba = 15,
    }
    class Environment
    {
        public readonly static Map map = new Map();

        /// <summary> more value => best path  |  less value => best code eficience </summary>
        public readonly static float densityPath = 0.2f;
        /// <summary> count of nodes to visit(with bfs) after found the destination node </summary>
        public readonly static int bfsArea = 40;
        /// <summary> heuristic weigth for path between triangles </summary>
        public readonly static float heuristicTriangleWeigth = 1f;
        /// <summary> heuristic weigth for path between points </summary>
        public readonly static float heuristicPointWeigth = 1f;
        /// <summary> width for border collision</summary>
        public readonly static float collisionBorder = 1.1f;
        /// <summary> create path analizando the agents in this moment</summary>
        public readonly static bool pathWithAgents = false;
    }
    class Obsolete
    {
    }
}
