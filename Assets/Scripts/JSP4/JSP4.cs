using System;
using System.Collections.Generic;

public class JPS4
{
    private bool[,] grid;
    private List<Node> open = new List<Node>();
    private HashSet<(int,int)> closed = new HashSet<(int,int)>();

    private static readonly (int x,int y)[] DIR4 =
    {
        (0,-1),  // 上
        (1,0),   // 右
        (0,1),   // 下
        (-1,0)   // 左
    };

    public class Node
    {
        public int x, y;
        public float g, h;
        public Node parent;
        public float f => g + h;

        // JumpResult 继承来的跳点类型
        public JumpType jumpType;

        public Node(int x, int y) { this.x = x; this.y = y; }
    }

    public enum JumpType
    {
        NormalJump,
        ForcedStop,       // 遇到墙前的停止点
        ForcedNeighbor,   // 被迫拐弯的跳点
        Goal              // 命中目标
    }

    private class JumpResult
    {
        public int x, y;
        public JumpType type;
    }

    public JPS4(bool[,] grid)
    {
        this.grid = grid;
    }

    public List<(int,int)> FindPath(int sx, int sy, int tx, int ty)
    {
        open.Clear();
        closed.Clear();

        Node start = new Node(sx, sy) { g=0, h=Heuristic(sx,sy,tx,ty), parent=null };
        open.Add(start);

        while (open.Count > 0)
        {
            open.Sort((a,b)=>a.f.CompareTo(b.f));
            Node cur = open[0];
            open.RemoveAt(0);

            if (cur.x == tx && cur.y == ty)
                return BuildPath(cur);

            closed.Add((cur.x, cur.y));
            IdentifySuccessors(cur, tx, ty);
        }

        return null;
    }

    private void IdentifySuccessors(Node node, int tx, int ty)
    {
        var dirs = GetDirections(node);

        foreach (var (dx,dy) in dirs)
        {
            var res = Jump(node.x, node.y, dx, dy, tx, ty);
            if (res == null) continue;

            if (closed.Contains((res.x, res.y)))
                continue;

            float newG = node.g + Distance(node.x, node.y, res.x, res.y);

            Node exist = open.Find(n => n.x == res.x && n.y == res.y);
            if (exist == null)
            {
                Node jp = new Node(res.x, res.y);
                jp.jumpType = res.type;       // ★ 关键：把跳点类型传到节点
                jp.g = newG;
                jp.h = Heuristic(jp.x, jp.y, tx, ty);
                jp.parent = node;
                open.Add(jp);
            }
            else if (newG < exist.g)
            {
                exist.g = newG;
                exist.parent = node;
                exist.jumpType = res.type;    // ★ 同样更新跳点类型
            }
        }
    }

    private JumpResult Jump(int x, int y, int dx, int dy, int tx, int ty)
    {
        int nx = x + dx;
        int ny = y + dy;

        // 1. forcedStop → 下一步是墙
        if (!Walkable(nx, ny))
        {
            return new JumpResult { x = x, y = y, type = JumpType.ForcedStop };
        }

        // 2. goal
        if (nx == tx && ny == ty)
        {
            return new JumpResult { x = nx, y = ny, type = JumpType.Goal };
        }

        // 3. forcedNeighbor
        if (HasForcedNeighbor(nx, ny, dx, dy))
        {
            return new JumpResult { x=nx, y=ny, type = JumpType.ForcedNeighbor };
        }

        // 4. 继续跳
        return Jump(nx, ny, dx, dy, tx, ty);
    }

    private bool HasForcedNeighbor(int x, int y, int dx, int dy)
    {
        if (dx != 0)
        {
            if (!Walkable(x, y-1) && Walkable(x+dx, y-1)) return true;
            if (!Walkable(x, y+1) && Walkable(x+dx, y+1)) return true;
        }
        else
        {
            if (!Walkable(x-1, y) && Walkable(x-1, y+dy)) return true;
            if (!Walkable(x+1, y) && Walkable(x+1, y+dy)) return true;
        }
        return false;
    }

    /*
       ★ 关键修正：
         node.jumpType 直接影响扩展方向，而不是无脑沿父方向或依赖 node==parent 判断
    */
    private List<(int dx,int dy)> GetDirections(Node node)
    {
        var dirs = new List<(int,int)>();

        if (node.parent == null)
        {
            dirs.AddRange(DIR4);
            return dirs;
        }

        // 获取父方向
        int dx = Math.Sign(node.x - node.parent.x);
        int dy = Math.Sign(node.y - node.parent.y);

        switch (node.jumpType)
        {
            case JumpType.NormalJump:
            case JumpType.ForcedNeighbor:
            case JumpType.Goal:
                // 正常跳点/forcedNeighbor/goal → 继续直走
                dirs.Add((dx,dy));
                return dirs;

            case JumpType.ForcedStop:
                // forcedStop（撞墙跳点） → 转向正交方向
                if (dx != 0)
                {
                    dirs.Add((0,-1));
                    dirs.Add((0,+1));
                }
                else if (dy != 0)
                {
                    dirs.Add((+1,0));
                    dirs.Add((-1,0));
                }
                return dirs;
        }

        return dirs;
    }

    // 常规辅助函数
    private bool Walkable(int x, int y)
    {
        if (x<0 || y<0 || x>=grid.GetLength(0) || y>=grid.GetLength(1))
            return false;
        return !grid[x,y];
    }

    private float Heuristic(int x,int y,int tx,int ty)
        => Math.Abs(x-tx) + Math.Abs(y-ty);

    private float Distance(int x1, int y1, int x2, int y2)
        => Math.Abs(x1-x2) + Math.Abs(y1-y2);

    private List<(int,int)> BuildPath(Node n)
    {
        var path = new List<(int,int)>();
        while (n != null)
        {
            path.Add((n.x, n.y));
            n = n.parent;
        }
        path.Reverse();
        return path;
    }
}
