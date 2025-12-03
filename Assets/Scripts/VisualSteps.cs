namespace PathFinding
{
    public interface IVisualStep
    {
        void Execute();
    }

    public abstract class VisualStep : IVisualStep
    {
        protected Tile _tile;

        public VisualStep(Tile tile)
        {
            _tile = tile;
        }

        public abstract void Execute();
    }

    public class MarkStartTileStep : VisualStep
    {
        public MarkStartTileStep(Tile tile) : base(tile)
        {
        }

        public override void Execute()
        {
            _tile.SetColor(_tile.Grid.TileColor_Start);
            _tile.SetText("S", 60);
        }
    }

    public class MarkEndTileStep : VisualStep
    {
        public MarkEndTileStep(Tile tile) : base(tile)
        {
        }

        public override void Execute()
        {
            _tile.SetColor(_tile.Grid.TileColor_End);
            _tile.SetText("E", 60);
        }
    }

    public class MarkPathTileStep : VisualStep
    {
        public MarkPathTileStep(Tile tile) : base(tile)
        {
        }

        public override void Execute()
        {
            _tile.SetColor(_tile.Grid.TileColor_Path);
            _tile.SetText(_tile.Cost.ToString(), 60);
        }
    }

    public class PushTileInFrontierStep : VisualStep
    {
        private int _cost;
        private float _hx;

        public PushTileInFrontierStep(Tile tile, int cost) : base(tile)
        {
            _cost = cost;
        }
        
        public PushTileInFrontierStep(Tile tile, int cost, float hx) : base(tile)
        {
            _cost = cost;
            _hx = hx;
        }

        public override void Execute()
        {
            _tile.SetColor(_tile.Grid.TileColor_Frontier);
            
            if(_hx > 0)
                _tile.SetText(_cost != 0 ? $"{_cost}/{_hx:N0}\n{_cost+_hx:N0}" : "", 40);
            else
                _tile.SetText(_cost != 0 ? _cost.ToString() : "");
            
        }
    }

    public class VisitTileStep : VisualStep
    {
        public VisitTileStep(Tile tile) : base(tile)
        {
        }

        public override void Execute()
        {
            _tile.SetColor(_tile.Grid.TileColor_Visited);
            // 保留Cost的显示
            if (_tile.Cost > 0)
            {
                _tile.SetText(_tile.Cost.ToString());
            }
        }
    }
}
