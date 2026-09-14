#pragma once
#include <stdint.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════
//  Maze Module
//
//  Stores the 16×16 maze as a grid of MazeCell structs.
//  Each cell records which of its four sides have a wall and
//  a flood-fill distance value used for navigation.
//
//  Coordinate system:
//    (0,0) = bottom-left corner  (robot start)
//    +X    = East
//    +Y    = North
//
//  Wall bits (stored per-cell as the cell's own boundary):
//    WALL_N  0x01  — wall on the north  edge of this cell
//    WALL_E  0x02  — wall on the east   edge
//    WALL_S  0x04  — wall on the south  edge
//    WALL_W  0x08  — wall on the west   edge
//    VISITED 0x10  — this cell has been physically entered
//
//  Heading constants (also used as direction indices 0-3):
//    NORTH 0 | EAST 1 | SOUTH 2 | WEST 3
// ═══════════════════════════════════════════════════════════════

// Wall bit masks
#define WALL_N   0x01
#define WALL_E   0x02
#define WALL_S   0x04
#define WALL_W   0x08
#define VISITED  0x10

// Heading values — MUST stay 0–3 for array indexing
#define NORTH  0
#define EAST   1
#define SOUTH  2
#define WEST   3

struct MazeCell {
    uint8_t walls;  // Wall bitmask (lower nibble) + visited flag (bit 4)
    uint8_t flood;  // BFS distance to goal (0 = goal, 255 = unreachable)
};

// The full maze grid — readable from webserver & navigator
extern MazeCell g_maze[MAZE_SIZE][MAZE_SIZE];

// ── API ──────────────────────────────────────────────────────

// Reset maze: clear all walls, set perimeter walls, flood = 255
void maze_init();

// Add a wall on the given side of (x,y).
// Automatically sets the mirror wall on the adjacent cell.
void maze_set_wall(int x, int y, uint8_t wallBit);

// Query whether a wall exists on a specific side of a cell.
// Returns true for out-of-bounds (treated as solid wall).
bool maze_has_wall(int x, int y, uint8_t wallBit);

// Mark / check visited status
void maze_mark_visited(int x, int y);
bool maze_is_visited(int x, int y);

// BFS flood fill from (goalX, goalY).
// Sets every reachable cell's .flood to its shortest-path distance.
// After calling this, navigate toward decreasing flood values.
void maze_flood_fill(int goalX, int goalY);

// Return the direction (NORTH/EAST/SOUTH/WEST) of the neighbour
// with the lowest flood value.
//   respectWalls = true  → only consider open passages (normal nav)
//   respectWalls = false → ignore walls (planning / display only)
uint8_t maze_best_direction(int x, int y, bool respectWalls);
