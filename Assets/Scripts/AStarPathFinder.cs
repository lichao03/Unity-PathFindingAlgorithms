using System;
using System.Collections.Generic;

namespace PathFinding
{
    /// <summary>
    /// A*寻路算法实现
    /// A*算法是一种启发式搜索算法，结合了Dijkstra算法的最短路径保证和贪婪最佳优先搜索的效率
    /// 它使用估价函数 f(n) = g(n) + h(n)，其中：
    /// - g(n) 是从起点到当前节点的实际代价
    /// - h(n) 是从当前节点到终点的启发式估计代价
    /// </summary>
    public class AStarPathFinder : PathFinderBase
    {
        public override List<Tile> FindPath(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            // 添加起点和终点的可视化标记
            AddStartEndMarkers(start, end, outSteps);

            // 初始化所有格子的代价为最大值（表示未访问）
            foreach (var tile in grid.Tiles)
            {
                tile.Cost = int.MaxValue;
            }

            // 起点的实际代价 g(n) = 0
            start.Cost = 0;

            // 定义启发式比较函数：f(n) = g(n) + h(n)
            // 这是A*算法的核心，通过估价函数来选择最有希望的节点
            Comparison<Tile> heuristicComparison = (lhs, rhs) =>
            {
                // 左节点的总代价 = 实际代价 + 启发式代价
                float lhsCost = lhs.Cost + GetEuclideanHeuristicCost(lhs, end);
                // 右节点的总代价 = 实际代价 + 启发式代价
                float rhsCost = rhs.Cost + GetEuclideanHeuristicCost(rhs, end);

                // 返回比较结果，小的优先级高
                return lhsCost.CompareTo(rhsCost);
            };

            // 创建优先队列（最小堆），使用启发式比较函数
            // 优先队列会自动按照 f(n) 值从小到大排序
            MinHeap<Tile> frontier = new MinHeap<Tile>(heuristicComparison);
            frontier.Add(start);

            // 已访问集合，防止重复访问同一节点
            HashSet<Tile> visited = new HashSet<Tile>();
            visited.Add(start);

            // 起点没有前驱节点
            start.PrevTile = null;

            // 主循环：当优先队列不为空时继续搜索
            while (frontier.Count > 0)
            {
                // 从优先队列中取出 f(n) 值最小的节点
                // 这是A*算法的关键：总是优先探索最有希望的节点
                Tile current = frontier.Remove();

                // 可视化：标记当前访问的节点（排除起点和终点）
                if (current != start && current != end)
                {
                    outSteps.Add(new VisitTileStep(current));
                }

                // 找到终点，搜索结束
                if (current == end)
                {
                    break;
                }

                // 遍历当前节点的所有邻居（上下左右四个方向）
                foreach (var neighbor in grid.GetNeighbors(current))
                {
                    // 计算从起点经过当前节点到达邻居的新代价
                    // newNeighborCost = g(current) + weight(neighbor)
                    int newNeighborCost = current.Cost + neighbor.Weight;
                    
                    // 如果找到了更短的路径，更新邻居的代价和前驱节点
                    // 这确保了找到的路径是最优的
                    if (newNeighborCost < neighbor.Cost)
                    {
                        neighbor.Cost = newNeighborCost;  // 更新 g(n)
                        neighbor.PrevTile = current;      // 更新前驱节点，用于路径回溯
                    }

                    // 如果邻居还没有被访问过
                    if (!visited.Contains(neighbor))
                    {
                        // 将邻居加入优先队列，等待后续探索
                        // 优先队列会根据 f(n) = g(n) + h(n) 自动排序
                        frontier.Add(neighbor);
                        visited.Add(neighbor);

                        // 可视化：显示邻居加入到边界队列中（排除终点）
                        if (neighbor != end)
                        {
                            outSteps.Add(new PushTileInFrontierStep(neighbor, neighbor.Cost));
                        }
                    }
                }
            }

            // 从终点回溯到起点，构建完整路径
            List<Tile> path = BacktrackToPath(end);
            
            // 可视化：标记最终路径
            AddPathVisualization(path, start, end, outSteps);

            return path;
        }
    }
}
