using System;
using System.Collections.Generic;
using UnityEngine;

namespace PathFinding
{
    /// <summary>
    /// JPS跳点搜索算法实现（仅支持4个方向）
    /// JPS是A*算法的优化版本，通过识别"跳点"来跳过对称路径上的冗余节点
    /// 核心思想：在空旷区域不逐格搜索，而是沿直线"跳跃"到关键点
    /// 相比A*可减少40%-90%的节点访问量
    /// </summary>
    public class JPSPathFinder : PathFinderBase
    {
        public override List<Tile> FindPath(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            // 添加起点和终点的可视化标记
            AddStartEndMarkers(start, end, outSteps);

            // 初始化所有格子的代价为最大值
            foreach (var tile in grid.Tiles)
            {
                tile.Cost = int.MaxValue;
                tile.PrevTile = null;
            }

            start.Cost = 0;

            // 使用A*的评估函数
            Comparison<Tile> heuristicComparison = (lhs, rhs) =>
            {
                float lhsCost = lhs.Cost + GetManhattanHeuristicCost(lhs, end);
                float rhsCost = rhs.Cost + GetManhattanHeuristicCost(rhs, end);
                return lhsCost.CompareTo(rhsCost);
            };

            MinHeap<Tile> openSet = new MinHeap<Tile>(heuristicComparison);
            openSet.Add(start);

            HashSet<Tile> closedSet = new HashSet<Tile>();

            // 主循环
            while (openSet.Count > 0)
            {
                Tile current = openSet.Remove();

                // 可视化当前节点
                if (current != start && current != end)
                {
                    outSteps.Add(new VisitTileStep(current));
                }

                // 找到终点
                if (current == end)
                {
                    break;
                }

                closedSet.Add(current);

                // 识别后继跳点
                IdentifySuccessors(grid, current, end, openSet, closedSet, outSteps);
            }

            // 回溯路径
            List<Tile> path = BacktrackToPath(end);
            AddPathVisualization(path, start, end, outSteps);

            return path;
        }

        /// <summary>
        /// 识别当前节点的所有后继跳点
        /// </summary>
        private void IdentifySuccessors(TileGrid grid, Tile current, Tile end,
            MinHeap<Tile> openSet, HashSet<Tile> closedSet, List<IVisualStep> outSteps)
        {
            // 获取当前节点的搜索方向（经过剪枝）
            List<Vector2Int> directions = GetPrunedNeighbors(grid, current);

            foreach (var dir in directions)
            {
                // 沿方向跳跃，寻找跳点
                Tile jumpPoint = Jump(grid, current.Row, current.Col, dir.x, dir.y, end);

                if (jumpPoint != null && !closedSet.Contains(jumpPoint))
                {
                    // 计算到达跳点的曼哈顿距离代价
                    int dx = Mathf.Abs(jumpPoint.Col - current.Col);
                    int dy = Mathf.Abs(jumpPoint.Row - current.Row);
                    int moveCost = dx + dy; // 只有直线移动，代价为曼哈顿距离
                    
                    int newCost = current.Cost + moveCost * jumpPoint.Weight;

                    // 如果找到更优路径
                    if (newCost < jumpPoint.Cost)
                    {
                        jumpPoint.Cost = newCost;
                        jumpPoint.PrevTile = current;
                        
                        openSet.Add(jumpPoint);

                        // 可视化跳点
                        if (jumpPoint != end)
                        {
                            outSteps.Add(new PushTileInFrontierStep(jumpPoint, jumpPoint.Cost, 
                                GetManhattanHeuristicCost(jumpPoint, end)));
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 跳跃函数：沿指定方向寻找跳点（仅支持4个方向）
        /// </summary>
        private Tile Jump(TileGrid grid, int row, int col, int dx, int dy, Tile end)
        {
            // 下一个位置
            int nextRow = row + dy;
            int nextCol = col + dx;

            // 边界检查和障碍物检查
            Tile next = grid.GetTile(nextRow, nextCol);
            if (next == null || next.Weight >= 1000)
            {
                return null;
            }

            // 到达终点
            if (next == end)
            {
                return next;
            }

            // 检查是否有强制邻居
            if (HasForcedNeighbors(grid, nextRow, nextCol, dx, dy))
            {
                return next; // 这是一个跳点
            }

            // 继续沿当前方向跳跃
            return Jump(grid, nextRow, nextCol, dx, dy, end);
        }

        /// <summary>
        /// 检查是否有强制邻居
        /// 强制邻居的定义：由于障碍物的存在，必须经过当前节点才能到达的邻居
        /// </summary>
        private bool HasForcedNeighbors(TileGrid grid, int row, int col, int dx, int dy)
        {
            // 水平移动
            if (dx != 0 && dy == 0)
            {
                // 检查上方：如果侧面有障碍，前方对角可走，则有强制邻居
                Tile upBlock = grid.GetTile(row - 1, col - dx);
                Tile upNext = grid.GetTile(row - 1, col);
                if ((upBlock == null || upBlock.Weight >= 1000) && 
                    upNext != null && upNext.Weight < 1000)
                {
                    return true;
                }

                // 检查下方
                Tile downBlock = grid.GetTile(row + 1, col - dx);
                Tile downNext = grid.GetTile(row + 1, col);
                if ((downBlock == null || downBlock.Weight >= 1000) && 
                    downNext != null && downNext.Weight < 1000)
                {
                    return true;
                }
            }
            // 垂直移动
            else if (dx == 0 && dy != 0)
            {
                // 检查左侧
                Tile leftBlock = grid.GetTile(row - dy, col - 1);
                Tile leftNext = grid.GetTile(row, col - 1);
                if ((leftBlock == null || leftBlock.Weight >= 1000) && 
                    leftNext != null && leftNext.Weight < 1000)
                {
                    return true;
                }

                // 检查右侧
                Tile rightBlock = grid.GetTile(row - dy, col + 1);
                Tile rightNext = grid.GetTile(row, col + 1);
                if ((rightBlock == null || rightBlock.Weight >= 1000) && 
                    rightNext != null && rightNext.Weight < 1000)
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// 获取经过剪枝的邻居方向（仅4个方向）
        /// JPS的核心优化：根据父节点方向进行对称路径剪枝
        /// </summary>
        private List<Vector2Int> GetPrunedNeighbors(TileGrid grid, Tile current)
        {
            List<Vector2Int> neighbors = new List<Vector2Int>();
            Tile parent = current.PrevTile;

            // 没有父节点（起点）：探索所有4个方向
            if (parent == null)
            {
                // 上
                if (IsWalkable(grid, current.Row - 1, current.Col))
                {
                    neighbors.Add(new Vector2Int(0, -1));
                }
                // 下
                if (IsWalkable(grid, current.Row + 1, current.Col))
                {
                    neighbors.Add(new Vector2Int(0, 1));
                }
                // 左
                if (IsWalkable(grid, current.Row, current.Col - 1))
                {
                    neighbors.Add(new Vector2Int(-1, 0));
                }
                // 右
                if (IsWalkable(grid, current.Row, current.Col + 1))
                {
                    neighbors.Add(new Vector2Int(1, 0));
                }
            }
            else
            {
                // 有父节点：基于移动方向进行剪枝
                int dx = Mathf.Clamp(current.Col - parent.Col, -1, 1);
                int dy = Mathf.Clamp(current.Row - parent.Row, -1, 1);

                // 水平移动
                if (dx != 0 && dy == 0)
                {
                    // 1. 继续前进方向
                    if (IsWalkable(grid, current.Row, current.Col + dx))
                    {
                        neighbors.Add(new Vector2Int(dx, 0));
                    }

                    // 2. 检查强制邻居方向
                    // 检查上方是否有强制邻居
                    Tile upBlock = grid.GetTile(current.Row - 1, current.Col - dx);
                    if ((upBlock == null || upBlock.Weight >= 1000) && 
                        IsWalkable(grid, current.Row - 1, current.Col))
                    {
                        neighbors.Add(new Vector2Int(0, -1));
                    }

                    // 检查下方是否有强制邻居
                    Tile downBlock = grid.GetTile(current.Row + 1, current.Col - dx);
                    if ((downBlock == null || downBlock.Weight >= 1000) && 
                        IsWalkable(grid, current.Row + 1, current.Col))
                    {
                        neighbors.Add(new Vector2Int(0, 1));
                    }
                }
                // 垂直移动
                else if (dx == 0 && dy != 0)
                {
                    // 1. 继续前进方向
                    if (IsWalkable(grid, current.Row + dy, current.Col))
                    {
                        neighbors.Add(new Vector2Int(0, dy));
                    }

                    // 2. 检查强制邻居方向
                    // 检查左侧是否有强制邻居
                    Tile leftBlock = grid.GetTile(current.Row - dy, current.Col - 1);
                    if ((leftBlock == null || leftBlock.Weight >= 1000) && 
                        IsWalkable(grid, current.Row, current.Col - 1))
                    {
                        neighbors.Add(new Vector2Int(-1, 0));
                    }

                    // 检查右侧是否有强制邻居
                    Tile rightBlock = grid.GetTile(current.Row - dy, current.Col + 1);
                    if ((rightBlock == null || rightBlock.Weight >= 1000) && 
                        IsWalkable(grid, current.Row, current.Col + 1))
                    {
                        neighbors.Add(new Vector2Int(1, 0));
                    }
                }
            }

            return neighbors;
        }

        /// <summary>
        /// 检查指定位置是否可行走
        /// </summary>
        private bool IsWalkable(TileGrid grid, int row, int col)
        {
            Tile tile = grid.GetTile(row, col);
            return tile != null && tile.Weight < 40;
        }
    }
}

