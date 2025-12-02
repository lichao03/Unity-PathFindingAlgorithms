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

        public int Rows;
        public int Cols;
        public GameObject TilePrefab;

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
        private bool _isPaused = false;
        private bool _stepNext = false;
        private bool _isRunning = false;
        private int _currentStep = 0;
        private int _totalSteps = 0;

        private void Awake()
        {
            Tiles = new Tile[Rows * Cols];
            for (int r = 0; r < Rows; r++)
            {
                for (int c = 0; c < Cols; c++)
                {
                    Tile tile = new Tile(this, r, c, TileWeight_Default);
                    tile.InitGameObject(transform, TilePrefab);

                    int index = GetTileIndex(r, c);
                    Tiles[index] = tile;
                }
            }


            CreateExpensiveArea(3, 3, 9, 1, TileWeight_Expensive);
            CreateExpensiveArea(3, 11, 1, 9, TileWeight_Expensive);
            ResetGrid();
        }

        private void Update()
        {
            Tile start = GetTile(9, 2);
            Tile end = GetTile(7, 14);

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
                start.SetColor(TileColor_Start);
                end.SetColor(TileColor_End);
            }
        }

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
            bool rowInRange = row >= 0 && row < Rows;
            bool colInRange = col >= 0 && col < Cols;
            return rowInRange && colInRange;
        }

        private int GetTileIndex(int row, int col)
        {
            return row * Cols + col;
        }
    }
}
