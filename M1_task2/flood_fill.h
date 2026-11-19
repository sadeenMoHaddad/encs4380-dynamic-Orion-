#ifndef FLOOD_FILL_H
#define FLOOD_FILL_H

#include <stdint.h>
#include <stdbool.h>

/* Maximum maze dimensions supported by the ENCS4380 mouse HAL. */
#define FF_MAX_SIZE 16

/* Direction numbering matches mouse_hal.h: north=0, east=1, south=2, west=3. */
enum {
    FF_NORTH = 0,
    FF_EAST  = 1,
    FF_SOUTH = 2,
    FF_WEST  = 3
};

/* Wall bits match MM_WALL(dir) in mouse_hal.h. */
#define FF_WALL(dir) ((uint8_t)(0x8u >> ((unsigned)(dir) & 3u)))

/*
 * Compute distances from every reachable cell to the goal region.
 *
 * walls[x][y]: wall bits for cell (x,y).
 * known[x][y]: sides that have actually been sensed.
 * Goal bounds are inclusive and may describe one cell or a larger rectangle.
 * dist[x][y]: output distance; -1 means no route under the selected policy.
 *
 * ff_compute():
 *   Uses only passages confirmed open from BOTH cells sharing the passage.
 *   Use this for shortest paths through the known map.
 *
 * ff_compute_explore():
 *   Uses an optimistic policy: an edge is blocked if either side records a wall.
 *   Otherwise, unknown sides are treated as potentially open.
 *   Use this when selecting cells to explore in an unknown maze.
 *
 * Both functions use iterative BFS with a fixed-size queue (no recursion).
 * Return true for valid input and false for invalid dimensions/goal bounds
 * or null pointers.
 */
bool ff_compute(int width, int height,
                const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                int gx0, int gy0, int gx1, int gy1,
                int dist[FF_MAX_SIZE][FF_MAX_SIZE]);

bool ff_compute_explore(int width, int height,
                        const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                        const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                        int gx0, int gy0, int gx1, int gy1,
                        int dist[FF_MAX_SIZE][FF_MAX_SIZE]);

/*
 * Choose one neighboring direction whose distance is strictly smaller.
 * Tie order: straight, right, left, then back relative to current_heading.
 *
 * ff_next_direction() only uses passages confirmed open from both cells.
 * ff_next_direction_explore() also permits unknown passages unless either
 * cell records a wall.
 *
 * Returns FF_NORTH..FF_WEST, or -1 if no decreasing neighbor exists,
 * the current cell is at the goal, or the input is invalid.
 */
int ff_next_direction(int width, int height,
                      const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                      const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                      const int dist[FF_MAX_SIZE][FF_MAX_SIZE],
                      int x, int y, int current_heading);

int ff_next_direction_explore(int width, int height,
                              const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                              const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                              const int dist[FF_MAX_SIZE][FF_MAX_SIZE],
                              int x, int y, int current_heading);

#endif
