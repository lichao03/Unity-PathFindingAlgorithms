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
        /// <summary>
        /// 跳跃结果类，包含跳点位置和类型
        /// </summary>
        private class JumpResult
        {
            public Tile Tile { get; set; }
            public JumpType Type { get; set; }
        }

        /// <summary>
        /// 跳跃缓存：key=(row, col, dx, dy)，避免重复计算相同位置和方向的跳跃结果
        /// 大幅提升性能，特别是在复杂地图中
        /// </summary>
        private Dictionary<(int row, int col, int dx, int dy), JumpResult> jumpMemo;

        /// <summary>
        /// 已访问节点集合，用于 Early Termination 优化
        /// </summary>
        private HashSet<Tile> closedSet;

        public override List<Tile> FindPath(TileGrid grid, Tile start, Tile end, List<IVisualStep> outSteps)
        {
            // ★ 初始化缓存和已访问集合
            jumpMemo = new Dictionary<(int, int, int, int), JumpResult>();
            closedSet = new HashSet<Tile>();

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
                IdentifySuccessors(grid, current, end, openSet, outSteps);
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
            MinHeap<Tile> openSet, List<IVisualStep> outSteps)
        {
            // 获取当前节点的搜索方向（经过剪枝）
            List<Vector2Int> directions = GetPrunedNeighbors(grid, current);

            foreach (var dir in directions)
            {
                // 沿方向跳跃，寻找跳点
                JumpResult result = Jump(grid, current.Row, current.Col, dir.x, dir.y, end, outSteps);

                if (result == null || result.Tile == null) continue;
                if (closedSet.Contains(result.Tile)) continue;
                
                // 计算到达跳点的曼哈顿距离代价
                int dx = Mathf.Abs(result.Tile.Col - current.Col);
                int dy = Mathf.Abs(result.Tile.Row - current.Row);
                int moveCost = dx + dy; // 只有直线移动，代价为曼哈顿距离
                    
                int newCost = current.Cost + moveCost * result.Tile.Weight;
                
                // 如果找到更优路径
                if (newCost < result.Tile.Cost)
                {
                    result.Tile.Cost = newCost;
                    result.Tile.PrevTile = current;
                    result.Tile.JumpType = result.Type; // ★ 保存跳点类型
                        
                    openSet.Add(result.Tile);

                    // 可视化跳点
                    if (result.Tile != end)
                    {
                        outSteps.Add(new PushTileInFrontierStep(result.Tile, result.Tile.Cost, 
                            GetManhattanHeuristicCost(result.Tile, end)));
                    }
                }
            }
        }

        /// <summary>
        /// 跳跃函数：沿指定方向寻找跳点（仅支持4个方向）
        /// 返回JumpResult包含跳点和类型信息
        /// ★ 优化：添加缓存和提前终止机制
        /// </summary>
        private JumpResult Jump(TileGrid grid, int row, int col, int dx, int dy, Tile end, List<IVisualStep> outSteps)
        {
            // ★ 优化1：缓存查询 - 避免重复计算相同位置和方向的跳跃
            var key = (row, col, dx, dy);
            if (jumpMemo.TryGetValue(key, out var cached))
            {
                return cached;
            }

            // 计算下一个位置
            int nextRow = row + dy;
            int nextCol = col + dx;

            // 1. Early Termination: 下一步是墙或边界，当前位置是停止点（ForcedStop）
            Tile next = grid.GetTile(nextRow, nextCol);
            if (next == null || !next.IsWalkable())
            {
                Tile currentTile = grid.GetTile(row, col);
                var result = new JumpResult { Tile = currentTile, Type = JumpType.ForcedStop };
                jumpMemo[key] = result; // ★ 缓存结果
                return result;
            }

            // 2. Goal: 到达终点
            if (next == end)
            {
                var result = new JumpResult { Tile = next, Type = JumpType.Goal };
                jumpMemo[key] = result; // ★ 缓存结果
                return result;
            }

            // 3. ForcedNeighbor: 检查是否有强制邻居
            if (HasForcedNeighbors(grid, nextRow, nextCol, dx, dy))
            {
                var result = new JumpResult { Tile = next, Type = JumpType.ForcedNeighbor };
                jumpMemo[key] = result; // ★ 缓存结果
                return result;
            }

            // ★ 优化2：Early Termination - 前方节点已探索过，且不可能产生新跳点
            if (closedSet.Contains(next))
            {
                jumpMemo[key] = null; // ★ 缓存 null 结果
                return null;
            }

            // 4. 可视化跳过的节点
            outSteps.Add(new JumpOverStep(next));
            
            // 5. 继续沿当前方向跳跃（递归）
            var jumpResult = Jump(grid, nextRow, nextCol, dx, dy, end, outSteps);
            jumpMemo[key] = jumpResult; // ★ 缓存递归结果
            return jumpResult;
        }

        /// <summary>
        /// 检查是否有强制邻居
        /// 强制邻居的定义：由于障碍物的存在，必须经过当前节点才能到达的邻居
        /// </summary>
        private bool HasForcedNeighbors(TileGrid grid, int row, int col, int dx, int dy)
        {
            // 水平移动（dx≠0, dy=0）
            if (dx != 0 && dy == 0)
            {
                // 上方被挡住 但上前方可走
                if (!IsWalkable(grid, row - 1, col) && IsWalkable(grid, row - 1, col + dx))
                {
                    return true;
                }

                // 下方被挡住 但下前方可走
                if (!IsWalkable(grid, row + 1, col) && IsWalkable(grid, row + 1, col + dx))
                {
                    return true;
                }
            }
            // 垂直移动（dx=0, dy≠0）
            else if (dx == 0 && dy != 0)
            {
                // 左方被挡住 但左前方可走
                if (!IsWalkable(grid, row, col - 1) && IsWalkable(grid, row + dy, col - 1))
                {
                    return true;
                }

                // 右方被挡住 但右前方可走
                if (!IsWalkable(grid, row, col + 1) && IsWalkable(grid, row + dy, col + 1))
                {
                    return true;
                }
            }

            return false;
        }


        /// <summary>
        /// 获取经过剪枝的邻居方向（仅4个方向）
        /// ★ 关键修正：根据跳点类型决定扩展方向
        /// Vector2Int(dx, dy): x=列变化(dx), y=行变化(dy)
        /// </summary>
        private List<Vector2Int> GetPrunedNeighbors(TileGrid grid, Tile current)
        {
            List<Vector2Int> neighbors = new List<Vector2Int>();
            Tile parent = current.PrevTile;

            // 没有父节点（起点）：探索所有4个方向
            if (parent == null)
            {
                // 上 (Row-1)
                if (IsWalkable(grid, current.Row - 1, current.Col))
                {
                    neighbors.Add(new Vector2Int(0, -1)); // dx=0, dy=-1
                }
                // 下 (Row+1)
                if (IsWalkable(grid, current.Row + 1, current.Col))
                {
                    neighbors.Add(new Vector2Int(0, 1)); // dx=0, dy=+1
                }
                // 左 (Col-1)
                if (IsWalkable(grid, current.Row, current.Col - 1))
                {
                    neighbors.Add(new Vector2Int(-1, 0)); // dx=-1, dy=0
                }
                // 右 (Col+1)
                if (IsWalkable(grid, current.Row, current.Col + 1))
                {
                    neighbors.Add(new Vector2Int(1, 0)); // dx=+1, dy=0
                }
                return neighbors;
            }

            // 计算父方向
            int dx = Math.Sign(current.Col - parent.Col);
            int dy = Math.Sign(current.Row - parent.Row);

            // ★ 根据跳点类型决定扩展方向
            switch (current.JumpType)
            {
                case JumpType.NormalJump:
                case JumpType.ForcedNeighbor:
                case JumpType.Goal:
                    // 正常跳点/强制邻居/目标点 → 继续沿父方向前进
                    neighbors.Add(new Vector2Int(dx, dy));
                    break;

                case JumpType.ForcedStop:
                    // 撞墙停止点 → 转向正交方向（垂直于当前方向）
                    if (dx != 0) // 水平移动遇墙 → 转向上下
                    {
                        if (IsWalkable(grid, current.Row - 1, current.Col))
                            neighbors.Add(new Vector2Int(0, -1)); // 上
                        if (IsWalkable(grid, current.Row + 1, current.Col))
                            neighbors.Add(new Vector2Int(0, 1));  // 下
                    }
                    else if (dy != 0) // 垂直移动遇墙 → 转向左右
                    {
                        if (IsWalkable(grid, current.Row, current.Col - 1))
                            neighbors.Add(new Vector2Int(-1, 0)); // 左
                        if (IsWalkable(grid, current.Row, current.Col + 1))
                            neighbors.Add(new Vector2Int(1, 0));  // 右
                    }
                    break;
            }

            return neighbors;
        }

        /// <summary>
        /// 检查指定位置是否可行走
        /// </summary>
        private bool IsWalkable(TileGrid grid, int row, int col)
        {
            Tile tile = grid.GetTile(row, col);
            return tile != null && tile.IsWalkable();
        }
    }
}
