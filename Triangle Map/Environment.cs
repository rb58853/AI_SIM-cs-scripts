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
        public readonly static float densityPath = 0.5f;
        /// <summary> count of nodes to visit(with bfs) after found the destination node </summary>
        public readonly static int bfsArea = 40;
        /// <summary> heuristic weigth for path between triangles </summary>
        public readonly static float heuristicTriangleWeigth = 1f;
        /// <summary> width for border collision</summary>
        public readonly static float collisionBorder = 1.2f;
        /// <summary> create path analizando the agents in this moment</summary>
        public readonly static bool pathWithAgents = false;
        /// <summary> Dilate the path of triangles</summary>
        public readonly static int trianglePathDilatation = 1;
        /// <summary> Length of view agent detected collisions </summary>
        public readonly static float viewLenAgent = 3f;
        /// <summary> Freq of review collisions for agents, update frequence = (freq/[speed / 5]) frames </summary>
        public readonly static int freqReview = 10;

      
        /// <summary> Draw path of agents </summary>
        public readonly static bool drawPaths = false;
        public readonly static bool drawAllPossiblePaths = false;
    }
    class Obsolete
    {
    }
}
