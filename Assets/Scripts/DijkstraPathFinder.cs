using System.Collections.Generic;

namespace PathFinding
{
    public class DijkstraPathFinder : PathFinderBase
    {
        public override List<Tile> FindPath(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            AddStartEndMarkers(start, end, outSteps);

            foreach (var tile in grid.Tiles)
            {
                tile.Cost = int.MaxValue;
            }

            start.Cost = 0;

            HashSet<Tile> visited = new HashSet<Tile>();
            visited.Add(start);

            MinHeap<Tile> frontier = new MinHeap<Tile>((lhs, rhs) => lhs.Cost.CompareTo(rhs.Cost));
            frontier.Add(start);

            start.PrevTile = null;

            while (frontier.Count > 0)
            {
                Tile current = frontier.Remove();
                
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
                    int newNeighborCost = current.Cost + neighbor.Weight;
                    if (newNeighborCost < neighbor.Cost)
                    {
                        neighbor.Cost = newNeighborCost;
                        neighbor.PrevTile = current;
                    }

                    if (!visited.Contains(neighbor))
                    {
                        visited.Add(neighbor);
                        frontier.Add(neighbor);

                        if (neighbor != end)
                        {
                            outSteps.Add(new PushTileInFrontierStep(neighbor, neighbor.Cost));
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

