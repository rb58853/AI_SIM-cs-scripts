using BaseNode;

namespace Agent_Space
{
    public enum Material { basic = 10 }
    class Environment
    {
        public static Map map = new Map();
        public static float densityPath = 3f;
        public static int bfsArea = 100;
        public static float heuristicTriangleWeigth = 2f;
    }

}
