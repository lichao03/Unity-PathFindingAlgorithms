using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathFinding
{
    /// <summary>
    /// 寻路算法可视化网格管理器
    /// 负责网格的初始化、寻路算法的执行控制、以及可视化展示
    /// </summary>
    public class TileGrid : MonoBehaviour
    {
        #region 常量定义
        
        private const int TileWeight_Default = 1;
        private const int TileWeight_Expensive = 50;
        private const int TileWeight_Infinity = int.MaxValue;
        
        #endregion

        #region 公共配置字段

        [Header("地图设置")]
        [Tooltip("地图大小 (x=列数, y=行数)")]
        public Vector2Int MapSize = new Vector2Int(15, 15);
        public GameObject TilePrefab;

        [Header("起点和终点设置")]
        [Tooltip("起点位置 (x=列号, y=行号，从0开始)")]
        public Vector2Int StartPosition = new Vector2Int(2, 9);
        [Tooltip("终点位置 (x=列号, y=行号，从0开始)")]
        public Vector2Int EndPosition = new Vector2Int(14, 7);

        [Header("颜色设置")]
        public Color TileColor_Default = new Color(0.86f, 0.83f, 0.83f);
        public Color TileColor_Expensive = new Color(0.19f, 0.65f, 0.43f);
        public Color TileColor_Infinity = new Color(0.37f, 0.37f, 0.37f);
        public Color TileColor_Start = Color.green;
        public Color TileColor_End = Color.red;
        public Color TileColor_Path = new Color(0.73f, 0.0f, 1.0f);
        public Color TileColor_Visited = new Color(0.75f, 0.55f, 0.38f);
        public Color TileColor_Frontier = new Color(0.4f, 0.53f, 0.8f);

        #endregion

        #region 公共属性

        /// <summary>
        /// 所有网格格子的数组
        /// </summary>
        public Tile[] Tiles { get; private set; }

        #endregion

        #region 私有字段

        // 寻路控制相关
        private IEnumerator _pathRoutine;
        private bool _isPaused;
        private bool _stepNext;
        private bool _isRunning;
        private int _currentStep;
        private int _totalSteps;

        #endregion

        #region Unity生命周期方法

        /// <summary>
        /// 初始化网格和格子
        /// </summary>
        private void Awake()
        {
            InitializeGrid();
            CreateObstacleAreas();
            ValidateStartEndPositions();
            ResetGrid();
        }

        /// <summary>
        /// 处理用户输入，触发寻路算法
        /// </summary>
        private void Update()
        {
            HandlePathfindingInput();
        }

        /// <summary>
        /// 绘制控制UI界面
        /// </summary>
        private void OnGUI()
        {
            if (!_isRunning) return;
            
            DrawControlButtons();
            DrawStatusInfo();
        }

#if UNITY_EDITOR
        /// <summary>
        /// 编辑器中参数修改时的验证和更新
        /// </summary>
        private void OnValidate()
        {
            if (!Application.isPlaying || Tiles == null || Tiles.Length == 0)
                return;

            ValidateStartEndPositions();

            if (!_isRunning)
            {
                ResetGrid();
            }
        }
#endif

        #endregion

        #region 网格初始化

        /// <summary>
        /// 初始化网格，创建所有格子
        /// </summary>
        private void InitializeGrid()
        {
            int totalTiles = MapSize.x * MapSize.y;
            Tiles = new Tile[totalTiles];

            for (int row = 0; row < MapSize.y; row++)
            {
                for (int col = 0; col < MapSize.x; col++)
                {
                    Tile tile = new Tile(this, row, col, TileWeight_Default);
                    tile.InitGameObject(transform, TilePrefab);
                    Tiles[GetTileIndex(row, col)] = tile;
                }
            }
        }

        /// <summary>
        /// 创建障碍物区域（高代价区域）
        /// </summary>
        private void CreateObstacleAreas()
        {
            CreateExpensiveArea(3, 3, 9, 1, TileWeight_Expensive);
            CreateExpensiveArea(3, 11, 1, 9, TileWeight_Expensive);
        }

        /// <summary>
        /// 创建指定区域的高代价格子
        /// </summary>
        private void CreateExpensiveArea(int row, int col, int width, int height, int weight)
        {
            for (int r = row; r < row + height; r++)
            {
                for (int c = col; c < col + width; c++)
                {
                    Tile tile = GetTile(r, c);
                    if (tile != null)
                    {
                        tile.Weight = weight;
                    }
                }
            }
        }

        #endregion

        #region 输入处理

        /// <summary>
        /// 处理寻路算法的输入控制
        /// </summary>
        private void HandlePathfindingInput()
        {
            Tile start = GetStartTile();
            Tile end = GetEndTile();

            // 验证起点终点有效性
            if (start == null || end == null)
            {
                ValidateStartEndPositions();
                start = GetStartTile();
                end = GetEndTile();
            }

            // 选择寻路算法
            if (Input.GetKeyDown(KeyCode.Alpha1))
            {
                StartPathfinding(start, end, PathFinder.FindPath_BFS);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha2))
            {
                StartPathfinding(start, end, PathFinder.FindPath_Dijkstra);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha3))
            {
                StartPathfinding(start, end, PathFinder.FindPath_AStar);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha4))
            {
                StartPathfinding(start, end, PathFinder.FindPath_GreedyBestFirstSearch);
            }
            else if (Input.GetKeyDown(KeyCode.Escape))
            {
                StopPathfinding();
                ResetGrid();
            }
        }

        #endregion

        #region 寻路控制

        /// <summary>
        /// 启动寻路算法
        /// </summary>
        private void StartPathfinding(Tile start, Tile end, Func<TileGrid, Tile, Tile, List<IVisualStep>, List<Tile>> pathFindingFunc)
        {
            StopPathfinding();
            _pathRoutine = ExecutePathfinding(start, end, pathFindingFunc);
            StartCoroutine(_pathRoutine);
        }

        /// <summary>
        /// 停止当前寻路
        /// </summary>
        private void StopPathfinding()
        {
            if (_pathRoutine != null)
            {
                StopCoroutine(_pathRoutine);
                _pathRoutine = null;
            }
            
            _isRunning = false;
            _isPaused = false;
            _stepNext = false;
            _currentStep = 0;
        }

        /// <summary>
        /// 执行寻路算法的协程
        /// </summary>
        private IEnumerator ExecutePathfinding(Tile start, Tile end, Func<TileGrid, Tile, Tile, List<IVisualStep>, List<Tile>> pathFindingFunc)
        {
            // 初始化状态
            ResetGrid();
            _isRunning = true;
            _isPaused = true;
            _stepNext = false;
            _currentStep = 0;

            // 执行寻路算法，获取所有可视化步骤
            List<IVisualStep> steps = new List<IVisualStep>();
            pathFindingFunc(this, start, end, steps);
            _totalSteps = steps.Count;

            // 逐步执行可视化
            foreach (var step in steps)
            {
                // 等待用户操作（暂停或下一步）
                yield return new WaitUntil(() => !_isPaused || _stepNext);
                
                if (_stepNext)
                {
                    _stepNext = false;
                }
                
                step.Execute();
                _currentStep++;
            }
            
            _isRunning = false;
        }

        #endregion

        #region 网格管理

        /// <summary>
        /// 重置网格到初始状态
        /// </summary>
        private void ResetGrid()
        {
            // 重置所有格子
            foreach (var tile in Tiles)
            {
                tile.Cost = 0;
                tile.PrevTile = null;
                tile.SetText("");

                // 根据权重设置颜色
                Color color = tile.Weight switch
                {
                    TileWeight_Default => TileColor_Default,
                    TileWeight_Expensive => TileColor_Expensive,
                    TileWeight_Infinity => TileColor_Infinity,
                    _ => TileColor_Default
                };
                tile.SetColor(color);
            }

            // 标记起点和终点
            MarkStartEndTiles();
        }

        /// <summary>
        /// 标记起点和终点的颜色
        /// </summary>
        private void MarkStartEndTiles()
        {
            Tile startTile = GetStartTile();
            Tile endTile = GetEndTile();
            
            if (startTile != null)
            {
                startTile.SetColor(TileColor_Start);
            }
            if (endTile != null)
            {
                endTile.SetColor(TileColor_End);
            }
        }

        #endregion

        #region 验证方法

        /// <summary>
        /// 验证并修正起点和终点的位置，确保它们在有效范围内且不重叠
        /// </summary>
        private void ValidateStartEndPositions()
        {
            // 限制起点和终点在有效范围内
            StartPosition.x = Mathf.Clamp(StartPosition.x, 0, MapSize.x - 1);
            StartPosition.y = Mathf.Clamp(StartPosition.y, 0, MapSize.y - 1);
            EndPosition.x = Mathf.Clamp(EndPosition.x, 0, MapSize.x - 1);
            EndPosition.y = Mathf.Clamp(EndPosition.y, 0, MapSize.y - 1);

            // 确保起点和终点不重叠
            if (StartPosition == EndPosition)
            {
                AdjustEndPosition();
            }
        }

        /// <summary>
        /// 调整终点位置，避免与起点重叠
        /// </summary>
        private void AdjustEndPosition()
        {
            if (EndPosition.x < MapSize.x - 1)
            {
                EndPosition.x++;
            }
            else if (EndPosition.y < MapSize.y - 1)
            {
                EndPosition.y++;
            }
            else if (EndPosition.x > 0)
            {
                EndPosition.x--;
            }
            else if (EndPosition.y > 0)
            {
                EndPosition.y--;
            }
        }

        #endregion

        #region UI绘制

        /// <summary>
        /// 绘制控制按钮
        /// </summary>
        private void DrawControlButtons()
        {
            GUIStyle buttonStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = 20,
                padding = new RectOffset(20, 20, 10, 10)
            };

            const float buttonWidth = 150;
            const float buttonHeight = 50;
            const float buttonSpacing = 10;
            const float startX = 10;
            const float startY = 10;

            // 暂停/继续按钮
            string pauseButtonText = _isPaused ? "继续 (C)" : "暂停 (P)";
            if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), pauseButtonText, buttonStyle))
            {
                _isPaused = !_isPaused;
                Debug.Log(_isPaused ? "Paused" : "Resumed");
            }

            // 下一步按钮（仅在暂停时可用）
            GUI.enabled = _isPaused;
            if (GUI.Button(new Rect(startX + buttonWidth + buttonSpacing, startY, buttonWidth, buttonHeight), "下一步 (N)", buttonStyle))
            {
                _stepNext = true;
            }
            GUI.enabled = true;
        }

        /// <summary>
        /// 绘制状态信息
        /// </summary>
        private void DrawStatusInfo()
        {
            GUIStyle statusStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 16,
                normal = { textColor = Color.white }
            };

            const float startX = 10;
            const float startY = 70; // buttonHeight + 20

            // 状态显示
            string status = _isPaused ? "状态: 已暂停" : "状态: 运行中";
            GUI.Label(new Rect(startX, startY, 300, 30), status, statusStyle);
            
            // 步数显示
            string stepInfo = $"步数: {_currentStep} / {_totalSteps}";
            GUI.Label(new Rect(startX, startY + 30, 300, 30), stepInfo, statusStyle);
        }

        #endregion

        #region 工具方法

        /// <summary>
        /// 获取起点格子
        /// </summary>
        private Tile GetStartTile()
        {
            return GetTile(StartPosition.y, StartPosition.x);
        }

        /// <summary>
        /// 获取终点格子
        /// </summary>
        private Tile GetEndTile()
        {
            return GetTile(EndPosition.y, EndPosition.x);
        }

        /// <summary>
        /// 根据行列获取格子
        /// </summary>
        public Tile GetTile(int row, int col)
        {
            return IsInBounds(row, col) ? Tiles[GetTileIndex(row, col)] : null;
        }

        /// <summary>
        /// 获取指定格子的所有相邻格子（上下左右）
        /// </summary>
        public IEnumerable<Tile> GetNeighbors(Tile tile)
        {
            // 右
            Tile right = GetTile(tile.Row, tile.Col + 1);
            if (right != null) yield return right;

            // 上
            Tile up = GetTile(tile.Row - 1, tile.Col);
            if (up != null) yield return up;

            // 左
            Tile left = GetTile(tile.Row, tile.Col - 1);
            if (left != null) yield return left;

            // 下
            Tile down = GetTile(tile.Row + 1, tile.Col);
            if (down != null) yield return down;
        }

        /// <summary>
        /// 判断行列是否在网格范围内
        /// </summary>
        private bool IsInBounds(int row, int col)
        {
            return row >= 0 && row < MapSize.y && col >= 0 && col < MapSize.x;
        }

        /// <summary>
        /// 根据行列计算一维数组索引
        /// </summary>
        private int GetTileIndex(int row, int col)
        {
            return row * MapSize.x + col;
        }

        #endregion
    }
}
