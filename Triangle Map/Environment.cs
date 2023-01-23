using BaseNode;
using System.Collections.Generic;

namespace Agent_Space
{
    public enum AgentType
    {
        normal,
        acuatic,
        fire
    }

    public enum Quality
    {
        low = 0,
        normal = 1,
        higth = 2,
    }

    public enum Material
    {
        basic = 10,
        fire = 40,
        water = 20,
        hierba = 13,
    }
    public class Environment
    {
        public readonly static Map map = new Map();

        /// <summary> more value => best path  |  less value => best code eficience </summary>
        public readonly static float densityPath = 0.3f;
        /// <summary> count of nodes to visit(with bfs) after found the destination node </summary>
        public readonly static int bfsArea = 60;
        /// <summary> heuristic weigth for path between triangles </summary>
        public readonly static float heuristicTriangleWeigth = 1f;
        /// <summary> Dilate the path of triangles</summary>
        public readonly static int trianglePathDilatation = 1;
        /// <summary> Length of view agent detected collisions [default value = 3f] </summary>
        public readonly static float viewLenAgent = 3f;
        /// <summary> 
        ///If sum of radius of two agents * this var is low than distance analize, else no analize [default value = 3f] 
        ///</summary>
        public readonly static float distanceAnalizeCollision = 3f;

        /// <summary> 
        /// Freq of review collisions for agents, update frequence = ([freqReview]/[speed/5]) frames 
        /// grosso modo 25-50 frames = 1s, ejemplo: freqReview = 25 con 50 de speed = 2.5 frames 
        /// => 10-20 revisiones por segundo 
        ///</summary>
        public readonly static int freqReview = 10;
        ///<summary> radius* ocupedArea are the ocuped triangles [default value = 2.2f] </summary>
        public readonly static float ocupedArea = 2.5f;

        ///<summary> Move from vertices to vertices in the same arist </summary>
        public readonly static bool moveInArist = true;
        ///<summary> Stop if not found a path from the current position </summary>
        public readonly static int stopInCollision = 30;
        ///<summary> Stop if not found a path from the current position </summary>
        public readonly static int stopCountForEmpty = int.MaxValue;

        /// <summary> Create border in runtime when collision [default value: true] </summary>
        public readonly static bool createBorder = false;
        ///<summary> Create points of path for border only in current triangle  [default value: true] </summary>
        public readonly static bool onlyTriangleBorder = true;
        ///<summary> higth quality => better path, low => better eficience </summary>
        public readonly static Quality qualityBorder = Quality.higth;
        ///<summary> 
        ///Create paths from dinamyc point to all arist of triangle that contains this point [default value: true] 
        ///</summary>
        public readonly static bool dinamycPointToAllArists = true;
        /// <summary> width for border collision [default value: 1.1f] </summary>
        public readonly static float collisionBorder = 1.1f;
        /// <summary> 
        /// calcular exactamente si hay colision, en caso de falso calcular colision a partir de
        /// cierto punto del recorrido [default value: false]
        ///</summary>
        public readonly static bool exactCollision = false;
        /// <summary> Detenerse si la proxima posicion es muy alta su distancia [default value: false] </summary>
        public readonly static bool stopOnPath = true;
        /// <summary> 
        /// Tolerancia a tomar un camino largo, si hay muchos materiales se recomienda valor alto(ex: 30),
        /// en caso de haber pocos materiales un valor bajo (ex:10)
        /// </summary>
        public readonly static float stopOnPathDistance = 30f;


        /// <summary> Densidad del camino que se muestra, mas densidad implica mas precision</summary>
        public readonly static int densityVisualPath = 10;

        /// <summary> Use metaheuristic [default value: true]</summary>
        public readonly static bool metaheuristic = true;
        /// <summary> 
        ///Perimetro maximo de las sudivisiones de los triangulos, para metaheuristic[default value: 30]
        ///</summary>
        public readonly static float maxPerTriang = 50;


        /// <summary> Draw path of agents </summary>
        public readonly static bool drawPaths = false;
        public readonly static bool drawBorder = false;
        public readonly static bool drawAllPossiblePaths = false;

        static internal class Interactive
        {
            public static List<Agent> grup = new List<Agent>();
            public static Dictionary<AgentType, List<Agent>> groupFromType = new Dictionary<AgentType, List<Agent>>();
            public static bool allGroupInMove = true;
            public static int countInStop = 0;
            public static float distanceToStop = 2.3f;
        }
    }
}
