using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathFinding
{
    /// <summary>
    /// 障碍区域配置
    /// </summary>
    [Serializable]
    public class ObstacleArea
    {
        [Tooltip("起始位置 (x=列号, y=行号)")]
        public Vector2Int StartPosition;
        [Tooltip("区域大小 (x=宽度, y=高度)")]
        public Vector2Int Size = Vector2Int.one;
        [Tooltip("格子权重（代价）")]
        public int Weight = 50;
    }

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
        public Vector2Int MapSize = new Vector2Int(16, 12);
        public GameObject TilePrefab;

        [Header("起点和终点设置")]
        [Tooltip("起点位置 (x=列号, y=行号，从0开始)")]
        public Vector2Int StartPosition = new Vector2Int(2, 9);
        [Tooltip("终点位置 (x=列号, y=行号，从0开始)")]
        public Vector2Int EndPosition = new Vector2Int(14, 7);

        [Header("障碍区域设置")]
        [Tooltip("障碍区域列表")]
        public List<ObstacleArea> ObstacleAreas = new List<ObstacleArea>
        {
            new ObstacleArea { StartPosition = new Vector2Int(3, 3), Size = new Vector2Int(9, 1), Weight = 50 },
            new ObstacleArea { StartPosition = new Vector2Int(11, 3), Size = new Vector2Int(1, 9), Weight = 50 }
        };

        [Header("颜色设置")]
        public Color TileColor_Default = new Color(0.86f, 0.83f, 0.83f);
        public Color TileColor_Expensive = new Color(0.19f, 0.65f, 0.43f);
        public Color TileColor_Infinity = new Color(0.37f, 0.37f, 0.37f);
        public Color TileColor_Start = Color.green;
        public Color TileColor_End = Color.red;
        public Color TileColor_Path = new Color(0.73f, 0.0f, 1.0f);
        public Color TileColor_Visited = new Color(0.75f, 0.55f, 0.38f);
        public Color TileColor_Frontier = new Color(0.4f, 0.53f, 0.8f);

        [Header("坐标轴设置")]
        [Tooltip("是否显示行号和列号")]
        public bool ShowAxisLabels = true;
        [Tooltip("坐标轴标签字体大小")]
        public int AxisLabelFontSize = 24;
        [Tooltip("坐标轴标签颜色")]
        public Color AxisLabelColor = Color.white;

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
        
        // 标记是否已初始化
        private bool _isInitialized;

        // 容器节点
        private Transform _tilesContainer;
        private Transform _axisLabelsContainer;

        #endregion

        #region Unity生命周期方法

        /// <summary>
        /// 初始化网格和格子
        /// </summary>
        private void Awake()
        {
            InitializeGrid();
            ApplyObstacleAreas();
            ValidateStartEndPositions();
            ResetGrid();
            _isInitialized = true;
        }

        /// <summary>
        /// 在编辑器和运行时都显示网格
        /// </summary>
        private void Start()
        {
            // 确保在运行时网格可见
            if (Tiles != null)
            {
                ResetGrid();
            }
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
            DrawAlgorithmSelectionButtons();
            
            if (_isRunning)
            {
                DrawControlButtons();
                DrawStatusInfo();
            }
        }

#if UNITY_EDITOR
        /// <summary>
        /// 编辑器中参数修改时的验证和更新
        /// </summary>
        private void OnValidate()
        {
            // 在编辑器模式下也初始化网格
            if (!Application.isPlaying)
            {
                // 延迟执行以确保所有字段都已设置
                UnityEditor.EditorApplication.delayCall += () =>
                {
                    if (this != null && TilePrefab != null)
                    {
                        InitializeGridInEditor();
                    }
                };
                return;
            }

            // 运行时的验证
            if (Tiles == null || Tiles.Length == 0)
                return;

            ValidateStartEndPositions();

            if (!_isRunning)
            {
                ApplyObstacleAreas();
                ResetGrid();
            }
        }

        /// <summary>
        /// 编辑器模式下初始化网格
        /// </summary>
        private void InitializeGridInEditor()
        {
            if (Application.isPlaying || TilePrefab == null)
                return;

            // 清理旧的格子
            ClearExistingTiles();

            // 初始化新的格子
            InitializeGrid();
            ApplyObstacleAreas();
            ValidateStartEndPositions();
            ResetGrid();
            _isInitialized = true;
        }

        /// <summary>
        /// 清理现有的格子对象
        /// </summary>
        private void ClearExistingTiles()
        {
            // 清理transform下的所有子对象
            while (transform.childCount > 0)
            {
                DestroyImmediate(transform.GetChild(0).gameObject);
            }
            
            Tiles = null;
        }
#endif

        #endregion

        #region 网格初始化

        /// <summary>
        /// 初始化网格，创建所有格子
        /// </summary>
        private void InitializeGrid()
        {
            // 创建容器节点
            CreateContainerNodes();

            int totalTiles = MapSize.x * MapSize.y;
            Tiles = new Tile[totalTiles];

            for (int row = 0; row < MapSize.y; row++)
            {
                for (int col = 0; col < MapSize.x; col++)
                {
                    Tile tile = new Tile(this, row, col, TileWeight_Default);
                    tile.InitGameObject(_tilesContainer, TilePrefab);
                    Tiles[GetTileIndex(row, col)] = tile;
                }
            }

            // 创建坐标轴标签
            if (ShowAxisLabels)
            {
                CreateAxisLabels();
            }
        }

        /// <summary>
        /// 创建容器节点
        /// </summary>
        private void CreateContainerNodes()
        {
            // 创建或获取 Tiles 容器
            GameObject tilesObj = GameObject.Find("Tiles");
            if (tilesObj == null || tilesObj.transform.parent != transform)
            {
                tilesObj = new GameObject("Tiles");
                tilesObj.transform.parent = transform;
                tilesObj.transform.localPosition = Vector3.zero;
            }
            _tilesContainer = tilesObj.transform;
            
            // 运行时清空 Tiles 容器下的所有子节点
            if (Application.isPlaying)
            {
                ClearContainerChildren(_tilesContainer);
            }

            // 创建或获取 AxisLabels 容器
            GameObject axisLabelsObj = GameObject.Find("AxisLabels");
            if (axisLabelsObj == null || axisLabelsObj.transform.parent != transform)
            {
                axisLabelsObj = new GameObject("AxisLabels");
                axisLabelsObj.transform.parent = transform;
                axisLabelsObj.transform.localPosition = Vector3.zero;
            }
            _axisLabelsContainer = axisLabelsObj.transform;
            
            // 运行时清空 AxisLabels 容器下的所有子节点
            if (Application.isPlaying)
            {
                ClearContainerChildren(_axisLabelsContainer);
            }
        }

        /// <summary>
        /// 清空容器下的所有子节点（运行时使用）
        /// </summary>
        private void ClearContainerChildren(Transform container)
        {
            if (container == null)
                return;
            
            // 先收集所有子对象到列表中
            List<GameObject> children = new List<GameObject>();
            for (int i = 0; i < container.childCount; i++)
            {
                children.Add(container.GetChild(i).gameObject);
            }
            
            // 然后统一删除
            foreach (GameObject child in children)
            {
                if (child != null)
                {
                    Destroy(child);
                }
            }
        }

        /// <summary>
        /// 创建坐标轴标签（行号和列号）
        /// </summary>
        private void CreateAxisLabels()
        {
            // 创建列号标签（顶部，X轴）
            for (int col = 0; col < MapSize.x; col++)
            {
                GameObject labelObj = new GameObject($"ColLabel_{col}");
                labelObj.transform.parent = _axisLabelsContainer;
                labelObj.transform.localPosition = new Vector3(col, 1f, 0);
                
                TextMesh textMesh = labelObj.AddComponent<TextMesh>();
                textMesh.text = col.ToString();
                textMesh.fontSize = AxisLabelFontSize;
                textMesh.color = AxisLabelColor;
                textMesh.anchor = TextAnchor.MiddleCenter;
                textMesh.alignment = TextAlignment.Center;
                textMesh.characterSize = 0.1f;
            }

            // 创建行号标签（左侧，Y轴）
            for (int row = 0; row < MapSize.y; row++)
            {
                GameObject labelObj = new GameObject($"RowLabel_{row}");
                labelObj.transform.parent = _axisLabelsContainer;
                labelObj.transform.localPosition = new Vector3(-1.5f, -row, 0);
                
                TextMesh textMesh = labelObj.AddComponent<TextMesh>();
                textMesh.text = row.ToString();
                textMesh.fontSize = AxisLabelFontSize;
                textMesh.color = AxisLabelColor;
                textMesh.anchor = TextAnchor.MiddleCenter;
                textMesh.alignment = TextAlignment.Center;
                textMesh.characterSize = 0.1f;
            }
        }

        /// <summary>
        /// 应用所有配置的障碍区域
        /// </summary>
        private void ApplyObstacleAreas()
        {
            if (Tiles == null || ObstacleAreas == null)
                return;

            // 首先重置所有格子权重为默认值
            foreach (var tile in Tiles)
            {
                if (tile != null)
                {
                    tile.Weight = TileWeight_Default;
                }
            }

            // 应用所有障碍区域
            foreach (var area in ObstacleAreas)
            {
                CreateExpensiveArea(area.StartPosition.y, area.StartPosition.x, 
                                   area.Size.x, area.Size.y, area.Weight);
            }
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
            // 只保留ESC重置功能
            if (Input.GetKeyDown(KeyCode.Escape))
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
                yield return new WaitUntil(() => !_isPaused || _stepNext||_currentStep<2);
                
                if (_stepNext)
                {
                    _stepNext = false;
                }
                
                step.Execute();
                _currentStep++;
                if (!_isPaused)
                    yield return new WaitForSeconds(0.3f);// 可调整的延时
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

        /// <summary>
        /// 绘制算法选择按钮
        /// </summary>
        private void DrawAlgorithmSelectionButtons()
        {
            GUIStyle buttonStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = 18,
                padding = new RectOffset(15, 15, 10, 10)
            };

            const float buttonWidth = 180;
            const float buttonHeight = 45;
            const float buttonSpacing = 8;
            float startX = Screen.width - buttonWidth - 10; // 右侧对齐
            float startY = 10;

            // 如果正在运行，禁用算法选择按钮
            GUI.enabled = !_isRunning;

            // BFS 算法按钮
            if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), "广度优先 (BFS)", buttonStyle))
            {
                StartPathfinding(GetStartTile(), GetEndTile(), PathFinder.FindPath_BFS);
            }
            startY += buttonHeight + buttonSpacing;

            // Dijkstra 算法按钮
            if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), "Dijkstra 算法", buttonStyle))
            {
                StartPathfinding(GetStartTile(), GetEndTile(), PathFinder.FindPath_Dijkstra);
            }
            startY += buttonHeight + buttonSpacing;

            // A* 算法按钮
            if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), "A* 算法", buttonStyle))
            {
                StartPathfinding(GetStartTile(), GetEndTile(), PathFinder.FindPath_AStar);
            }
            startY += buttonHeight + buttonSpacing;

            // Greedy 算法按钮
            if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), "贪婪最佳优先", buttonStyle))
            {
                StartPathfinding(GetStartTile(), GetEndTile(), PathFinder.FindPath_GreedyBestFirstSearch);
            }
            startY += buttonHeight + buttonSpacing;

            // JPS 算法按钮
            if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), "JPS 跳点搜索", buttonStyle))
            {
                StartPathfinding(GetStartTile(), GetEndTile(), PathFinder.FindPath_JPS);
            }
            startY += buttonHeight + buttonSpacing;

            GUI.enabled = true;

            // 重置按钮（始终可用）
            if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), "重置 (ESC)", buttonStyle))
            {
                StopPathfinding();
                ResetGrid();
            }
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
