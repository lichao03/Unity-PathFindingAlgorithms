using System;
using System.Collections.Generic;

public class JPS4Optimized
{
    private bool[,] grid;

    private List<Node> open = new List<Node>();
    private HashSet<(int, int)> closed = new HashSet<(int, int)>();

    private Dictionary<(int x,int y,int dx,int dy), JumpResult> jumpMemo =
        new Dictionary<(int,int,int,int), JumpResult>();

    private static readonly (int x, int y)[] DIR4 =
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

        public JumpType jumpType;

        public Node(int x, int y)
        {
            this.x = x; this.y = y;
        }
    }

    public enum JumpType
    {
        NormalJump,
        ForcedStop,
        ForcedNeighbor,
        Goal
    }

    public class JumpResult
    {
        public int x, y;
        public JumpType type;
    }

    public JPS4Optimized(bool[,] grid)
    {
        this.grid = grid;
    }

    public List<(int, int)> FindPath(int sx, int sy, int tx, int ty)
    {
        open.Clear();
        closed.Clear();
        jumpMemo.Clear();

        Node start = new Node(sx, sy);
        start.g = 0;
        start.h = Heuristic(sx, sy, tx, ty);
        open.Add(start);

        while (open.Count > 0)
        {
            open.Sort((a, b) => a.f.CompareTo(b.f));
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
        var dirs = GetDirections(node, tx, ty);

        foreach (var (dx, dy) in dirs)
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
                jp.jumpType = res.type;
                jp.g = newG;
                jp.h = Heuristic(jp.x, jp.y, tx, ty);
                jp.parent = node;
                open.Add(jp);
            }
            else if (newG < exist.g)
            {
                exist.g = newG;
                exist.parent = node;
                exist.jumpType = res.type;
            }
        }
    }

    private JumpResult Jump(int x, int y, int dx, int dy, int tx, int ty)
    {
        var key = (x, y, dx, dy);
        if (jumpMemo.TryGetValue(key, out var cached))
            return cached;

        int nx = x + dx;
        int ny = y + dy;

        // --- Early Termination ---
        if (!Walkable(nx, ny))
        {
            var r = new JumpResult { x = x, y = y, type = JumpType.ForcedStop };
            jumpMemo[key] = r;
            return r;
        }

        // goal
        if (nx == tx && ny == ty)
        {
            var r = new JumpResult { x = nx, y = ny, type = JumpType.Goal };
            jumpMemo[key] = r;
            return r;
        }

        // forced neighbor
        if (HasForcedNeighbor(nx, ny, dx, dy))
        {
            var r = new JumpResult { x = nx, y = ny, type = JumpType.ForcedNeighbor };
            jumpMemo[key] = r;
            return r;
        }

        // --- Early Termination 2: 前方节点已探索过，且不可能产生跳点 ---
        if (closed.Contains((nx, ny)))
        {
            jumpMemo[key] = null;
            return null;
        }

        // --- Jump recursion ---
        var next = Jump(nx, ny, dx, dy, tx, ty);
        jumpMemo[key] = next;
        return next;
    }

    private bool HasForcedNeighbor(int x, int y, int dx, int dy)
    {
        if (dx != 0) 
        {
            if (!Walkable(x, y - 1) && Walkable(x + dx, y - 1)) return true;
            if (!Walkable(x, y + 1) && Walkable(x + dx, y + 1)) return true;
        }
        else 
        {
            if (!Walkable(x - 1, y) && Walkable(x - 1, y + dy)) return true;
            if (!Walkable(x + 1, y) && Walkable(x + 1, y + dy)) return true;
        }
        return false;
    }

    private List<(int dx, int dy)> GetDirections(Node node, int tx, int ty)
    {
        var dirs = new List<(int, int)>();

        if (node.parent == null)
        {
            foreach (var d in DIR4)
            {
                if (DirectionTowardGoal(node.x, node.y, tx, ty, d.x, d.y))
                    dirs.Add(d);
            }
            return dirs;
        }

        int dx = Math.Sign(node.x - node.parent.x);
        int dy = Math.Sign(node.y - node.parent.y);

        switch (node.jumpType)
        {
            case JumpType.NormalJump:
            case JumpType.ForcedNeighbor:
            case JumpType.Goal:
                if (DirectionTowardGoal(node.x, node.y, tx, ty, dx, dy))
                    dirs.Add((dx, dy));
                return dirs;

            case JumpType.ForcedStop:
                if (dx != 0)
                {
                    if (DirectionTowardGoal(node.x, node.y, tx, ty, 0, -1)) dirs.Add((0, -1));
                    if (DirectionTowardGoal(node.x, node.y, tx, ty, 0, +1)) dirs.Add((0, +1));
                }
                else if (dy != 0)
                {
                    if (DirectionTowardGoal(node.x, node.y, tx, ty, +1, 0)) dirs.Add((+1, 0));
                    if (DirectionTowardGoal(node.x, node.y, tx, ty, -1, 0)) dirs.Add((-1, 0));
                }
                return dirs;
        }

        return dirs;
    }

    private bool DirectionTowardGoal(int x, int y, int tx, int ty, int dx, int dy)
    {
        if (dx > 0 && tx < x) return false;
        if (dx < 0 && tx > x) return false;
        if (dy > 0 && ty < y) return false;
        if (dy < 0 && ty > y) return false;
        return true;
    }

    private bool Walkable(int x, int y)
    {
        if (x < 0 || y < 0 || x >= grid.GetLength(0) || y >= grid.GetLength(1))
            return false;
        return !grid[x, y];
    }

    private float Heuristic(int x, int y, int tx, int ty)
        => Math.Abs(x - tx) + Math.Abs(y - ty);

    private float Distance(int x1, int y1, int x2, int y2)
        => Math.Abs(x1 - x2) + Math.Abs(y1 - y2);

    private List<(int, int)> BuildPath(Node n)
    {
        var path = new List<(int, int)>();
        while (n != null)
        {
            path.Add((n.x, n.y));
            n = n.parent;
        }
        path.Reverse();
        return path;
    }
}
