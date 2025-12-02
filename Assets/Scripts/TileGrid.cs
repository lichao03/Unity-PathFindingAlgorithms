using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PathFinding
{
    public class TileGrid : MonoBehaviour
    {
        private const int TileWeight_Default = 1;
        private const int TileWeight_Expensive = 50;
        private const int TileWeight_Infinity = int.MaxValue;

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

        public Tile[] Tiles { get; private set; }

        private IEnumerator _pathRoutine;
        private bool _isPaused;
        private bool _stepNext;
        private bool _isRunning;
        private int _currentStep;
        private int _totalSteps;

        private void Awake()
        {
            Tiles = new Tile[MapSize.y * MapSize.x];
            for (int r = 0; r < MapSize.y; r++)
            {
                for (int c = 0; c < MapSize.x; c++)
                {
                    Tile tile = new Tile(this, r, c, TileWeight_Default);
                    tile.InitGameObject(transform, TilePrefab);

                    int index = GetTileIndex(r, c);
                    Tiles[index] = tile;
                }
            }

            CreateExpensiveArea(3, 3, 9, 1, TileWeight_Expensive);
            CreateExpensiveArea(3, 11, 1, 9, TileWeight_Expensive);
            
            // Validate and clamp start/end positions
            ValidateStartEndPositions();
            
            ResetGrid();
        }

        private void Update()
        {
            // Get start and end tiles using public parameters
            Tile start = GetTile(StartPosition.y, StartPosition.x);
            Tile end = GetTile(EndPosition.y, EndPosition.x);

            // Validate positions in case they were changed in inspector
            if (start == null || end == null)
            {
                ValidateStartEndPositions();
                start = GetTile(StartPosition.y, StartPosition.x);
                end = GetTile(EndPosition.y, EndPosition.x);
            }

            if (Input.GetKeyDown(KeyCode.Alpha1))
            {
                StopPathCoroutine();
                _pathRoutine = FindPath(start, end, PathFinder.FindPath_BFS);
                StartCoroutine(_pathRoutine);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha2))
            {
                StopPathCoroutine();
                _pathRoutine = FindPath(start, end, PathFinder.FindPath_Dijkstra);
                StartCoroutine(_pathRoutine);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha3))
            {
                StopPathCoroutine();
                _pathRoutine = FindPath(start, end, PathFinder.FindPath_AStar);
                StartCoroutine(_pathRoutine);
            }
            else if (Input.GetKeyDown(KeyCode.Alpha4))
            {
                StopPathCoroutine();
                _pathRoutine = FindPath(start, end, PathFinder.FindPath_GreedyBestFirstSearch);
                StartCoroutine(_pathRoutine);
            }
            else if (Input.GetKeyDown(KeyCode.Escape))
            {
                StopPathCoroutine();
                ResetGrid();
            }
        }

        /// <summary>
        /// 验证并修正起点和终点的位置，确保它们在有效范围内
        /// </summary>
        private void ValidateStartEndPositions()
        {
            // Clamp start position to valid range
            StartPosition.y = Mathf.Clamp(StartPosition.y, 0, MapSize.y - 1);
            StartPosition.x = Mathf.Clamp(StartPosition.x, 0, MapSize.x - 1);

            // Clamp end position to valid range
            EndPosition.y = Mathf.Clamp(EndPosition.y, 0, MapSize.y - 1);
            EndPosition.x = Mathf.Clamp(EndPosition.x, 0, MapSize.x - 1);

            // Ensure start and end are not the same position
            if (StartPosition.y == EndPosition.y && StartPosition.x == EndPosition.x)
            {
                // Move end to a different position
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
        }

#if UNITY_EDITOR
        /// <summary>
        /// 在编辑器中修改参数时调用，用于实时更新起点终点显示
        /// </summary>
        private void OnValidate()
        {
            // Only process if in play mode and tiles are initialized
            if (!Application.isPlaying || Tiles == null || Tiles.Length == 0)
            {
                return;
            }

            // Validate positions
            ValidateStartEndPositions();

            // Update grid display if not running pathfinding
            if (!_isRunning)
            {
                ResetGrid();
            }
        }
#endif

        private void OnGUI()
        {
            // Only show control buttons when pathfinding is running
            if (!_isRunning) return;
            
            // Set GUI style for larger buttons
            GUIStyle buttonStyle = new GUIStyle(GUI.skin.button);
            buttonStyle.fontSize = 20;
            buttonStyle.padding = new RectOffset(20, 20, 10, 10);

            float buttonWidth = 150;
            float buttonHeight = 50;
            float buttonSpacing = 10;
            float startX = 10;
            float startY = 10;

            // Pause/Resume Toggle Button
            string pauseButtonText = _isPaused ? "继续 (C)" : "暂停 (P)";
            if (GUI.Button(new Rect(startX, startY, buttonWidth, buttonHeight), pauseButtonText, buttonStyle))
            {
                _isPaused = !_isPaused;
                Debug.Log(_isPaused ? "Paused" : "Resumed");
            }

            // Next Step Button (only enabled when paused)
            GUI.enabled = _isPaused;
            if (GUI.Button(new Rect(startX + buttonWidth + buttonSpacing, startY, buttonWidth, buttonHeight), "下一步 (N)", buttonStyle))
            {
                _stepNext = true;
            }
            GUI.enabled = true;

            // Status display
            GUIStyle statusStyle = new GUIStyle(GUI.skin.label);
            statusStyle.fontSize = 16;
            statusStyle.normal.textColor = Color.white;
            string status = _isPaused ? "状态: 已暂停" : "状态: 运行中";
            GUI.Label(new Rect(startX, startY + buttonHeight + 10, 300, 30), status, statusStyle);
            
            // Step counter display
            string stepInfo = $"步数: {_currentStep} / {_totalSteps}";
            GUI.Label(new Rect(startX, startY + buttonHeight + 40, 300, 30), stepInfo, statusStyle);
        }

        private void StopPathCoroutine()
        {
            if (_pathRoutine != null)
            {
                StopCoroutine(_pathRoutine);
                _pathRoutine = null;
                _isRunning = false;
                _isPaused = false;
                _stepNext = false;
                _currentStep = 0; // Reset step counter
            }
        }

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

        private void ResetGrid()
        {
            foreach (var tile in Tiles)
            {
                tile.Cost = 0;
                tile.PrevTile = null;
                tile.SetText("");

                switch (tile.Weight)
                {
                    case TileWeight_Default:
                        tile.SetColor(TileColor_Default);
                        break;
                    case TileWeight_Expensive:
                        tile.SetColor(TileColor_Expensive);
                        break;
                    case TileWeight_Infinity:
                        tile.SetColor(TileColor_Infinity);
                        break;
                }
            }

            // Set start and end colors
            Tile startTile = GetTile(StartPosition.y, StartPosition.x);
            Tile endTile = GetTile(EndPosition.y, EndPosition.x);
            if (startTile != null)
            {
                startTile.SetColor(TileColor_Start);
            }
            if (endTile != null)
            {
                endTile.SetColor(TileColor_End);
            }
        }

        private IEnumerator FindPath(Tile start, Tile end, Func<TileGrid, Tile, Tile, List<IVisualStep>, List<Tile>> pathFindingFunc)
        {
            ResetGrid();
            _isRunning = true;
            _isPaused = true;
            _stepNext = false;
            _currentStep = 0;

            List<IVisualStep> steps = new List<IVisualStep>();
            pathFindingFunc(this, start, end, steps);

            _totalSteps = steps.Count;

            foreach (var step in steps)
            {
                yield return new WaitUntil(()=> !_isPaused|| _stepNext);
                if (_stepNext)
                {
                    _stepNext = false;
                }
                step.Execute();

                _currentStep++;
            }
            
            _isRunning = false;
        }

        public Tile GetTile(int row, int col)
        {
            if (!IsInBounds(row, col))
            {
                return null;
            }

            return Tiles[GetTileIndex(row, col)];
        }

        public IEnumerable<Tile> GetNeighbors(Tile tile)
        {
            Tile right = GetTile(tile.Row, tile.Col + 1);
            if (right != null)
            {
                yield return right;
            }

            Tile up = GetTile(tile.Row - 1, tile.Col);
            if (up != null)
            {
                yield return up;
            }

            Tile left = GetTile(tile.Row, tile.Col - 1);
            if (left != null)
            {
                yield return left;
            }

            Tile down = GetTile(tile.Row + 1, tile.Col);
            if (down != null)
            {
                yield return down;
            }
        }

        private bool IsInBounds(int row, int col)
        {
            bool rowInRange = row >= 0 && row < MapSize.y;
            bool colInRange = col >= 0 && col < MapSize.x;
            return rowInRange && colInRange;
        }

        private int GetTileIndex(int row, int col)
        {
            return row * MapSize.x + col;
        }
    }
}
