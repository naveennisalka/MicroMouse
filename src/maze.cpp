/*
  maze.cpp — Maze Grid and Flood-Fill Solver

  The maze is stored as a 16×16 array of MazeCell.
  Each cell knows:
    - Which of its four sides have a confirmed wall (walls bitmask)
    - Its BFS flood-fill distance from the goal (flood value)

  The flood-fill uses BFS starting from the goal cell.
  After filling, the robot navigates by always stepping to the
  neighbour with the lowest flood value.

  Wall mirroring: when a wall is set on one side of a cell, the
  matching wall on the adjacent cell is automatically set too,
  keeping the map consistent.
*/
#include "maze.h"
#include <Arduino.h>

// ── Global maze data ─────────────────────────────────────────────
MazeCell g_maze[MAZE_SIZE][MAZE_SIZE];

// ── Direction lookup tables ──────────────────────────────────────
// Indexed by heading: NORTH=0, EAST=1, SOUTH=2, WEST=3
static const uint8_t kWallBit[4]  = { WALL_N, WALL_E, WALL_S, WALL_W };
static const uint8_t kOpposite[4] = { WALL_S, WALL_W, WALL_N, WALL_E };
static const int     kDX[4]       = {  0,  1,  0, -1 };  // +X = East
static const int     kDY[4]       = {  1,  0, -1,  0 };  // +Y = North

// ── maze_init ────────────────────────────────────────────────────
void maze_init() {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            g_maze[x][y].walls = 0;
            g_maze[x][y].flood = 255;
        }
    }

    // Perimeter walls (always solid in a standard micromouse maze)
    for (int i = 0; i < MAZE_SIZE; i++) {
        g_maze[i][0].walls           |= WALL_S;  // Bottom edge
        g_maze[i][MAZE_SIZE - 1].walls |= WALL_N; // Top edge
        g_maze[0][i].walls           |= WALL_W;  // Left edge
        g_maze[MAZE_SIZE - 1][i].walls |= WALL_E; // Right edge
    }
}

// ── maze_set_wall ─────────────────────────────────────────────────
void maze_set_wall(int x, int y, uint8_t wallBit) {
    if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return;

    g_maze[x][y].walls |= wallBit;

    // Mirror: find which direction this bit represents, then set
    // the opposite wall in the adjacent cell
    for (int d = 0; d < 4; d++) {
        if (kWallBit[d] != wallBit) continue;

        int nx = x + kDX[d];
        int ny = y + kDY[d];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            g_maze[nx][ny].walls |= kOpposite[d];
        }
        break;
    }
}

// ── maze_has_wall ─────────────────────────────────────────────────
bool maze_has_wall(int x, int y, uint8_t wallBit) {
    if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return true;
    return (g_maze[x][y].walls & wallBit) != 0;
}

// ── Visited helpers ───────────────────────────────────────────────
void maze_mark_visited(int x, int y) {
    if (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE)
        g_maze[x][y].walls |= VISITED;
}

bool maze_is_visited(int x, int y) {
    if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return false;
    return (g_maze[x][y].walls & VISITED) != 0;
}

// ── maze_flood_fill (BFS) ─────────────────────────────────────────
// Uses a simple array-based queue (max 256 cells for 16×16 maze)
struct Point { int x, y; };
static Point bfsQueue[MAZE_SIZE * MAZE_SIZE];

void maze_flood_fill(int goalX, int goalY) {
    // Reset all flood values to "unreachable"
    for (int y = 0; y < MAZE_SIZE; y++)
        for (int x = 0; x < MAZE_SIZE; x++)
            g_maze[x][y].flood = 255;

    // BFS from goal outward
    int head = 0, tail = 0;
    g_maze[goalX][goalY].flood = 0;
    bfsQueue[tail++] = { goalX, goalY };

    while (head < tail) {
        Point p     = bfsQueue[head++];
        uint8_t nxt = g_maze[p.x][p.y].flood + 1;

        for (int d = 0; d < 4; d++) {
            // Skip if there is a wall in this direction
            if (g_maze[p.x][p.y].walls & kWallBit[d]) continue;

            int nx = p.x + kDX[d];
            int ny = p.y + kDY[d];
            if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

            // Only update if we found a shorter path
            if (g_maze[nx][ny].flood <= nxt) continue;

            g_maze[nx][ny].flood = nxt;
            bfsQueue[tail++] = { nx, ny };
        }
    }
}

// ── maze_best_direction ───────────────────────────────────────────
uint8_t maze_best_direction(int x, int y, bool respectWalls) {
    uint8_t bestDir   = NORTH;
    uint8_t bestFlood = 255;

    for (int d = 0; d < 4; d++) {
        if (respectWalls && (g_maze[x][y].walls & kWallBit[d])) continue;

        int nx = x + kDX[d];
        int ny = y + kDY[d];
        if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;

        if (g_maze[nx][ny].flood < bestFlood) {
            bestFlood = g_maze[nx][ny].flood;
            bestDir   = (uint8_t)d;
        }
    }
    return bestDir;
}
