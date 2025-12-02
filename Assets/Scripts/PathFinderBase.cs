using System.Collections.Generic;
using UnityEngine;

namespace PathFinding
{
    public abstract class PathFinderBase
    {
        public abstract List<Tile> FindPath(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps);

        protected void AddStartEndMarkers(Tile start, Tile end, List<IVisualStep> outSteps)
        {
            outSteps.Add(new MarkStartTileStep(start));
            outSteps.Add(new MarkEndTileStep(end));
        }

        protected void AddPathVisualization(List<Tile> path, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            foreach (var tile in path)
            {
                if (tile == start || tile == end)
                {
                    continue;
                }

                outSteps.Add(new MarkPathTileStep(tile));
            }
        }

        protected float GetEuclideanHeuristicCost(Tile current, Tile end)
        {
            float heuristicCost = (current.ToVector2() - end.ToVector2()).magnitude;
            return heuristicCost;
        }

        protected List<Tile> BacktrackToPath(Tile end)
        {
            Tile current = end;
            List<Tile> path = new List<Tile>();

            while (current != null)
            {
                path.Add(current);
                current = current.PrevTile;
            }

            path.Reverse();

            return path;
        }
    }
}

