using BaseNode;

namespace Agent_Space
{
    public enum Quality
    {
        low = 0,
        normal = 1,
        higth = 2,
    }

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
        public readonly static int bfsArea = 20;
        /// <summary> heuristic weigth for path between triangles </summary>
        public readonly static float heuristicTriangleWeigth = 1f;
        /// <summary> create path analizando the agents in this moment</summary>
        public readonly static bool pathWithAgents = false;
        /// <summary> Dilate the path of triangles</summary>
        public readonly static int trianglePathDilatation = 1;
        /// <summary> Length of view agent detected collisions </summary>
        public readonly static float viewLenAgent = 2f;
        /// <summary> If sum of radius of two agents * this var is low than distance analize, else no analize </summary>
        public readonly static float distanceAnalizeCollision = 3f;
        /// <summary> Freq of review collisions for agents, update frequence = (freqReview/[speed]) frames </summary>
        public readonly static int freqReview = 30;
        ///<summary> radius* ocupedArea are the ocuped triangles </summary>
        public readonly static float ocupedArea = 3f;
        ///<summary> higth quality => better path, low => better eficience </summary>
        public readonly static Quality qualityBorder = Quality.higth;
        ///<summary> Move from vertices to vertices in the same arist </summary>
        public readonly static bool moveInArist = true;
        ///<summary> Stop if not found a path from the current position </summary>
        public readonly static int stopInCollision = 30;
        ///<summary> Stop if not found a path from the current position </summary>
        public readonly static int stopCountForEmpty = 50000;
        ///<summary> Create points of path for border only in current triangle</summary>

        public readonly static bool createBorder = true;
        /// <summary> width for border collision</summary>
        public readonly static bool onlyTriangleBorder = true;
        ///<summary> Create paths from dinamyc point to all arist of triangle that contains this point</summary>
        public readonly static bool dinamycPointToAllArists = true;
        /// <summary> Create border in runtime when collision</summary>
        public readonly static float collisionBorder = 1.1f;

        /// <summary> Densidad del camino que se muestra, mas densidad implica mas precision</summary>
        public readonly static int densityVisualPath = 50;
        // ///<summary> Stop if not found a path from the current position </summary>
        // public readonly static bool notFoundStop = true;

        /// <summary> Draw path of agents </summary>
        public readonly static bool drawPaths = false;
        public readonly static bool drawBorder = false;
        public readonly static bool drawAllPossiblePaths = false;
    }
    class Obsolete
    {
    }
}
