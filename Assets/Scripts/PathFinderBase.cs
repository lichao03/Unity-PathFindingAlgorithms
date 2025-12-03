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

        // 欧几里得启发式函数
        protected virtual float GetEuclideanHeuristicCost(Tile current, Tile end)
        {
            float heuristicCost = (current.ToVector2() - end.ToVector2()).magnitude;
            return heuristicCost;
        }
        
        // 曼哈顿启发式函数
        protected virtual float GetManhattanHeuristicCost(Tile current, Tile end)
        {
            var curVec = current.ToVector2();
            var endVec = end.ToVector2();
            float heuristicCost = Mathf.Abs(curVec.x - endVec.x) + Mathf.Abs(curVec.y - endVec.y);
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

