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
        /// <summary> Dilate the path of triangles</summary>
        public readonly static int trianglePathDilatation = 2;
        /// <summary> Length of view agent detected collisions </summary>
        public readonly static float viewLenAgent = 1.5f;
        /// <summary> If sum of radius of two agents * this var is low than distance analize, else no analize </summary>
        public readonly static float distanceAnalizeCollision = 2f;
        /// <summary> Freq of review collisions for agents, update frequence = (freqReview/[speed]) frames </summary>
        public readonly static int freqReview = 30;
        ///<summary> radius* ocupedArea are the ocuped triangles </summary>
        public readonly static float ocupedArea = 2.2f;
        ///<summary> higth quality => better path, low => better eficience </summary>
        public readonly static Quality qualityBorder = Quality.higth;
        ///<summary> Move from vertices to vertices in the same arist </summary>
        public readonly static bool moveInArist = true;
        ///<summary> Stop if not found a path from the current position </summary>
        public readonly static int stopInCollision = 30;
        ///<summary> Stop if not found a path from the current position </summary>
        public readonly static int stopCountForEmpty = 10;

        /// <summary> Create border in runtime when collision [default value: true] </summary>
        public readonly static bool createBorder = true;
        ///<summary> Create points of path for border only in current triangle  [default value: true] </summary>
        public readonly static bool onlyTriangleBorder = true;
        ///<summary> 
        ///Create paths from dinamyc point to all arist of triangle that contains this point [default value: true] 
        ///</summary>
        public readonly static bool dinamycPointToAllArists = true;
        /// <summary> width for border collision [default value: 1.1f] </summary>
        public readonly static float collisionBorder = 1.1f;
        /// <summary> Detenerse si la proxima posicion es muy alta [default value: false] </summary>
        public readonly static bool stopOnPath = false;
        public readonly static float stopOnPathDistance = 3f;


        /// <summary> Densidad del camino que se muestra, mas densidad implica mas precision</summary>
        public readonly static int densityVisualPath = 50;

        /// <summary> Use metaheuristic [default value: true]</summary>
        public readonly static bool metaheuristic = true;
        /// <summary> Perimetro maximo de las sudivisiones de los triangulos, para metaheuristic</summary>
        public readonly static float maxPerTriang = 12;


        /// <summary> Draw path of agents </summary>
        public readonly static bool drawPaths = false;
        public readonly static bool drawBorder = false;
        public readonly static bool drawAllPossiblePaths = false;
    }
    class Obsolete
    {
    }
}
