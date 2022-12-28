using BaseNode;

namespace Agent_Space
{
    public enum Material {
        basic = 10,
        fire = 1,
        water = 5
    }
    class Environment
    {
        public readonly static Map map = new Map();

        /// <summary> more value => best path  |  less value => best code eficience </summary>
        public readonly static float densityPath = 1f;
        /// <summary> count of nodes to visit(with bfs) after found the destination node </summary>
        public readonly static int bfsArea = 50;
        /// <summary> heuristic weigth for path between triangles </summary>
        public readonly static float heuristicTriangleWeigth = 2f;
    }

}
