using System.Collections.Generic;

namespace PathFinding
{
    public static class PathFinder
    {
        private static readonly BFSPathFinder _bfsPathFinder = new BFSPathFinder();
        private static readonly DijkstraPathFinder _dijkstraPathFinder = new DijkstraPathFinder();
        private static readonly AStarPathFinder _aStarPathFinder = new AStarPathFinder();
        private static readonly GreedyBestFirstSearchPathFinder _greedyPathFinder = new GreedyBestFirstSearchPathFinder();
        private static readonly JPSPathFinder _jpsPathFinder = new JPSPathFinder();

        public static List<Tile> FindPath_BFS(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            return _bfsPathFinder.FindPath(grid, start, end, outSteps);
        }

        public static List<Tile> FindPath_Dijkstra(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            return _dijkstraPathFinder.FindPath(grid, start, end, outSteps);
        }

        public static List<Tile> FindPath_AStar(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            return _aStarPathFinder.FindPath(grid, start, end, outSteps);
        }

        public static List<Tile> FindPath_GreedyBestFirstSearch(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            return _greedyPathFinder.FindPath(grid, start, end, outSteps);
        }

        public static List<Tile> FindPath_JPS(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            return _jpsPathFinder.FindPath(grid, start, end, outSteps);
        }
    }
}
