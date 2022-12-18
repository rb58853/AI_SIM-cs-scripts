using System;
using System.Collections.Generic;
using DijkstraSpace;

namespace MainSpace
{
    class Program
    {
        static void Main(string[] args)
        {
            cases();

        }
        static void cases()
        {
            TestCase(33, 1, 40);
            TestCase(95, 1, 100);
            TestCase(9, 99, 100);
            TestCase(95, 75, 100);
            TestCase(95, 95, 100);
            TestCase(186, 179, 190);
        }

        static void TestCase(int x, int y, int n)
        {
            DijkstraNode[,] grid = NewGrid(n, n);
            Dijkstra dijkstra = new Dijkstra(1, 1);
            dijkstra.GridToDijktraGraph(grid);
            dijkstra.Start(grid, grid[10, 10], grid[x, y]);

            foreach (DijkstraNode node in grid)
                node.ResetValuesToDefault();
            DijkstraA_star dijkstraA_star = new DijkstraA_star(1, 1);
            dijkstraA_star.Start(grid, grid[10, 10], grid[x, y]);
        }
        static DijkstraNode[,] NewGrid(int x, int y)
        {
            DijkstraNode[,] grid = new DijkstraNode[x, y];
            for (int i = 0; i < grid.GetLength(0); i++)
                for (int j = 0; j < grid.GetLength(1); j++)
                    grid[i, j] = new DijkstraNode();
            return grid;
        }
    }
    
}
