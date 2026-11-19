#include "flood_fill.h"

#define FF_CELL_COUNT (FF_MAX_SIZE * FF_MAX_SIZE)

typedef struct {
    int x;
    int y;
} FFCell;

/* Direction offsets: north, east, south, west. */
static const int dx[4] = { 0, 1, 0, -1 };
static const int dy[4] = { 1, 0, -1, 0 };

static bool ff_in_bounds(int width, int height, int x, int y)
{
    return x >= 0 && y >= 0 && x < width && y < height;
}

static int ff_opposite(int direction)
{
    return (direction + 2) % 4;
}

/*
 * Check a shared passage from both cells.
 *
 * Conservative mode:
 *   Both sides must be known and both must say the passage is open.
 *
 * Exploration mode:
 *   A wall recorded on either side blocks the passage. If neither side
 *   records a wall, the passage is considered potentially open.
 */
static bool ff_side_open(int width, int height,
                         const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                         const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                         int x, int y, int direction,
                         bool allow_unknown)
{
    int nx = x + dx[direction];
    int ny = y + dy[direction];
    int opposite = ff_opposite(direction);
    uint8_t bit = FF_WALL(direction);
    uint8_t opposite_bit = FF_WALL(opposite);

    if (!ff_in_bounds(width, height, nx, ny))
        return false;

    /* A known wall on either side always blocks the passage. */
    if (((known[x][y] & bit) != 0 && (walls[x][y] & bit) != 0) ||
        ((known[nx][ny] & opposite_bit) != 0 &&
         (walls[nx][ny] & opposite_bit) != 0))
        return false;

    if (allow_unknown)
        return true;

    /* For a confirmed route, both cells must know that the edge is open. */
    if ((known[x][y] & bit) == 0 ||
        (known[nx][ny] & opposite_bit) == 0)
        return false;

    return (walls[x][y] & bit) == 0 &&
           (walls[nx][ny] & opposite_bit) == 0;
}

static bool ff_valid_input(int width, int height,
                           const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                           const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                           int gx0, int gy0, int gx1, int gy1,
                           int dist[FF_MAX_SIZE][FF_MAX_SIZE])
{
    if (!walls || !known || !dist)
        return false;

    if (width < 1 || height < 1 ||
        width > FF_MAX_SIZE || height > FF_MAX_SIZE)
        return false;

    if (gx0 < 0 || gy0 < 0 ||
        gx1 < gx0 || gy1 < gy0 ||
        gx1 >= width || gy1 >= height)
        return false;

    return true;
}

static bool ff_compute_internal(
    int width, int height,
    const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
    const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
    int gx0, int gy0, int gx1, int gy1,
    int dist[FF_MAX_SIZE][FF_MAX_SIZE],
    bool allow_unknown)
{
    FFCell queue[FF_CELL_COUNT];
    int head = 0;
    int tail = 0;
    int x, y, d;

    if (!ff_valid_input(width, height, walls, known,
                        gx0, gy0, gx1, gy1, dist))
        return false;

    /* Step 1: initialize all cells as unreached. */
    for (x = 0; x < FF_MAX_SIZE; x++) {
        for (y = 0; y < FF_MAX_SIZE; y++)
            dist[x][y] = -1;
    }

    /* Step 2: every cell in the goal region starts at distance zero. */
    for (x = gx0; x <= gx1; x++) {
        for (y = gy0; y <= gy1; y++) {
            dist[x][y] = 0;
            queue[tail].x = x;
            queue[tail].y = y;
            tail++;
        }
    }

    /* Step 3: BFS expands through allowed neighboring passages. */
    while (head < tail) {
        FFCell current = queue[head++];

        for (d = FF_NORTH; d <= FF_WEST; d++) {
            int nx = current.x + dx[d];
            int ny = current.y + dy[d];

            if (ff_side_open(width, height, walls, known,
                             current.x, current.y, d, allow_unknown) &&
                dist[nx][ny] == -1) {
                dist[nx][ny] = dist[current.x][current.y] + 1;
                queue[tail].x = nx;
                queue[tail].y = ny;
                tail++;
            }
        }
    }

    return true;
}

bool ff_compute(int width, int height,
                const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                int gx0, int gy0, int gx1, int gy1,
                int dist[FF_MAX_SIZE][FF_MAX_SIZE])
{
    return ff_compute_internal(width, height, walls, known,
                                gx0, gy0, gx1, gy1, dist, false);
}

bool ff_compute_explore(int width, int height,
                        const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                        const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                        int gx0, int gy0, int gx1, int gy1,
                        int dist[FF_MAX_SIZE][FF_MAX_SIZE])
{
    return ff_compute_internal(width, height, walls, known,
                                gx0, gy0, gx1, gy1, dist, true);
}

static int ff_next_direction_internal(
    int width, int height,
    const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
    const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
    const int dist[FF_MAX_SIZE][FF_MAX_SIZE],
    int x, int y, int current_heading,
    bool allow_unknown)
{
    int order[4];
    int i;
    int current_distance;

    if (!walls || !known || !dist ||
        width < 1 || height < 1 ||
        width > FF_MAX_SIZE || height > FF_MAX_SIZE ||
        !ff_in_bounds(width, height, x, y) ||
        current_heading < FF_NORTH || current_heading > FF_WEST)
        return -1;

    current_distance = dist[x][y];

    if (current_distance <= 0)
        return -1;

    /* Prefer fewer turns when several decreasing choices are available. */
    order[0] = current_heading;
    order[1] = (current_heading + 1) % 4;
    order[2] = (current_heading + 3) % 4;
    order[3] = (current_heading + 2) % 4;

    for (i = 0; i < 4; i++) {
        int d = order[i];
        int nx = x + dx[d];
        int ny = y + dy[d];

        if (ff_side_open(width, height, walls, known,
                         x, y, d, allow_unknown) &&
            dist[nx][ny] >= 0 &&
            dist[nx][ny] < current_distance)
            return d;
    }

    return -1;
}

int ff_next_direction(int width, int height,
                      const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                      const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                      const int dist[FF_MAX_SIZE][FF_MAX_SIZE],
                      int x, int y, int current_heading)
{
    return ff_next_direction_internal(width, height, walls, known, dist,
                                       x, y, current_heading, false);
}

int ff_next_direction_explore(int width, int height,
                              const uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE],
                              const uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE],
                              const int dist[FF_MAX_SIZE][FF_MAX_SIZE],
                              int x, int y, int current_heading)
{
    return ff_next_direction_internal(width, height, walls, known, dist,
                                       x, y, current_heading, true);
}
