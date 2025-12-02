using System.Collections.Generic;

namespace PathFinding
{
    public class BFSPathFinder : PathFinderBase
    {
        public override List<Tile> FindPath(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            AddStartEndMarkers(start, end, outSteps);

            HashSet<Tile> visited = new HashSet<Tile>();
            visited.Add(start);

            Queue<Tile> frontier = new Queue<Tile>();
            frontier.Enqueue(start);

            start.PrevTile = null;

            while (frontier.Count > 0)
            {
                Tile current = frontier.Dequeue();

                if (current != start && current != end)
                {
                    outSteps.Add(new VisitTileStep(current));
                }

                if (current == end)
                {
                    break;
                }

                foreach (var neighbor in grid.GetNeighbors(current))
                {
                    if (!visited.Contains(neighbor))
                    {
                        visited.Add(neighbor);
                        frontier.Enqueue(neighbor);

                        neighbor.PrevTile = current;

                        if (neighbor != end)
                        {
                            outSteps.Add(new PushTileInFrontierStep(neighbor, 0));
                        }
                    }
                }
            }

            List<Tile> path = BacktrackToPath(end);
            AddPathVisualization(path, start, end, outSteps);

            return path;
        }
    }
}

