/*
 * solver.c (M1 Task 1) - Team T13 - Task 1: map and position
 *
 * Task 1 only, nothing else:
 *   1. a wall map, 4 bits per cell, written with MM_WALL(dir) only,
 *   2. every wall recorded on BOTH cells that share it,
 *   3. x, y and heading tracked after every move,
 *   4. maze size and goal read from mm_maze(),
 *   5. the map dump: mm_map_begin(), mm_map_cell() for every cell, mm_map_end().
 *
 * To fill the map, the mouse visits every cell it can reach with a simple
 * depth-first walk: go to any open neighbour not visited yet; when there is
 * none, drive back one cell towards the start. When it is back at the start
 * with nothing left, every reachable cell has been seen, so the dump is the
 * whole maze. (Task 2 replaces this walk with flood fill.)
 *
 * Build in mms with:
 *   gcc -O2 -std=c99 -o solver.exe main_sim.c hal_sim.c mm_common.c solver.c \
 *       flood_fill.c flood_fill_display.c
 * (flood_fill.* and flood_fill_display.* come from M1_task2; they show the
 *  flood-fill distance to the goal on every cell after it is sensed.)
 */
#include "mouse_hal.h"
#include "flood_fill.h"
#include "flood_fill_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define TEAM_ID        "T13"   
#define LOG_EVERY_CELL 1       /* 1 = log each reading and its conversion */

/* ==== Maze size and goal (from mm_maze, never constants) ================ */

static int g_w, g_h;                       /* maze width and height           */
static int g_gx0, g_gy0, g_gx1, g_gy1;     /* goal block, inclusive           */

/* ==== The map: 4 bits per cell ========================================== */
/* Arrays are sized for the largest maze the HAL supports (MM_MAX_SIZE);
 * only the first g_w x g_h cells are used. Index order is [x][y].        */

static uint8_t g_walls[MM_MAX_SIZE][MM_MAX_SIZE];   /* MM_WALL bits: wall here    */
static uint8_t g_known[MM_MAX_SIZE][MM_MAX_SIZE];   /* MM_WALL bits: side observed */
static int     g_dist[MM_MAX_SIZE][MM_MAX_SIZE];    /* flood fill distances        */
static bool    g_visited[MM_MAX_SIZE][MM_MAX_SIZE]; /* sensed from inside          */
static int8_t  g_back[MM_MAX_SIZE][MM_MAX_SIZE];    /* direction back to the cell
                                                       we came from (-1 at start) */
/* ==== The pose: where the mouse is and where it faces ================== */

static int g_x, g_y;     /* current cell                                    */
static int g_head;       /* MM_NORTH, MM_EAST, MM_SOUTH or MM_WEST          */
static int g_moves;      /* number of cells driven, for the log             */

/* ==== Direction helpers ================================================= */
/* mouse_hal.h: N=0, E=1, S=2, W=3; turning right adds 1 (mod 4). */

static int dir_dx(int d)    { return d == MM_EAST ? 1 : (d == MM_WEST ? -1 : 0); }
static int dir_dy(int d)    { return d == MM_NORTH ? 1 : (d == MM_SOUTH ? -1 : 0); }
static int dir_right(int d) { return (d + 1) % 4; }
static int dir_left(int d)  { return (d + 3) % 4; }
static int dir_back(int d)  { return (d + 2) % 4; }

static char dir_name(int d)
{
    switch (d) {
    case MM_NORTH: return 'N';
    case MM_EAST:  return 'E';
    case MM_SOUTH: return 'S';
    default:       return 'W';
    }
}

static bool in_maze(int x, int y) { return x >= 0 && y >= 0 && x < g_w && y < g_h; }

/* ==== 4. Maze size and goal ============================================= */

static bool maze_init(void)
{
    mm_maze_t m = mm_maze();

    g_w = m.width;
    g_h = m.height;
    g_gx0 = m.goal_x0; g_gy0 = m.goal_y0;
    g_gx1 = m.goal_x1; g_gy1 = m.goal_y1;

    if (g_w < 1 || g_h < 1 || g_w > MM_MAX_SIZE || g_h > MM_MAX_SIZE) {
        mm_log("task1: maze %dx%d is not supported", g_w, g_h);
        return false;
    }
    mm_log("task1: maze %dx%d, goal (%d,%d)-(%d,%d)",
           g_w, g_h, g_gx0, g_gy0, g_gx1, g_gy1);
    return true;
}

/* ==== 1 and 2. The wall map ============================================= */

/* Empty map. The outer wall is set in g_walls (it always exists) but not in
 * g_known: it counts as observed only once the mouse has really seen it. */
static void map_init(void)
{
    int x, y;
    memset(g_walls, 0, sizeof g_walls);
    memset(g_known, 0, sizeof g_known);
    memset(g_visited, 0, sizeof g_visited);
    for (x = 0; x < g_w; x++) {
        for (y = 0; y < g_h; y++) {
            g_back[x][y] = -1;
            if (y == g_h - 1) g_walls[x][y] |= MM_WALL(MM_NORTH);
            if (x == g_w - 1) g_walls[x][y] |= MM_WALL(MM_EAST);
            if (y == 0)       g_walls[x][y] |= MM_WALL(MM_SOUTH);
            if (x == 0)       g_walls[x][y] |= MM_WALL(MM_WEST);
        }
    }
}

/* Record that side d of cell (x,y) is a wall (true) or open (false).
 * A wall is shared: the east side of (x,y) IS the west side of (x+1,y).
 * So the neighbour gets the opposite side, dir_back(d), with the same value. */
static void map_set_side(int x, int y, int d, bool wall)
{
    int nx = x + dir_dx(d), ny = y + dir_dy(d);

    g_known[x][y] |= MM_WALL(d);
    if (wall) g_walls[x][y] |= MM_WALL(d);
    else      g_walls[x][y] &= (uint8_t)~MM_WALL(d);

    if (in_maze(nx, ny)) {                       /* the same wall, seen from the other cell */
        int back = dir_back(d);
        g_known[nx][ny] |= MM_WALL(back);
        if (wall) g_walls[nx][ny] |= MM_WALL(back);
        else      g_walls[nx][ny] &= (uint8_t)~MM_WALL(back);
    }

    if (wall) mm_show_wall(x, y, d);             /* draw it in the simulator */
}

/* Write "x,y" into buf (no stdio needed). */
static void cell_text(int x, int y, char *buf)
{
    int n = 0;
    if (x >= 10) buf[n++] = (char)('0' + x / 10);
    buf[n++] = (char)('0' + x % 10);
    buf[n++] = ',';
    if (y >= 10) buf[n++] = (char)('0' + y / 10);
    buf[n++] = (char)('0' + y % 10);
    buf[n] = '\0';
}

/* Read the three walls around the mouse and store them as absolute walls.
 * mm_sense() is RELATIVE to the mouse. With heading h:
 *   left  = (h + 3) % 4,  front = h,  right = (h + 1) % 4. */
static void sense_cell(void)
{
    uint8_t s = mm_sense();
    bool l = (s & MM_SEE_LEFT)  != 0;
    bool f = (s & MM_SEE_FRONT) != 0;
    bool r = (s & MM_SEE_RIGHT) != 0;
    int dl = dir_left(g_head), df = g_head, dr = dir_right(g_head);
    char txt[8];

    map_set_side(g_x, g_y, dl, l);
    map_set_side(g_x, g_y, df, f);
    map_set_side(g_x, g_y, dr, r);
    g_visited[g_x][g_y] = true;
    cell_text(g_x, g_y, txt);
    mm_label(g_x, g_y, txt);                     /* show the coordinates */

    ff_compute_explore(g_w, g_h, g_walls, g_known,
                       g_gx0, g_gy0, g_gx1, g_gy1, g_dist);
    ff_display_distances(g_w, g_h, g_dist);
#if LOG_EVERY_CELL
    mm_log("(%d,%d) facing %c: L=%d F=%d R=%d  ->  %c=%d %c=%d %c=%d",
           g_x, g_y, dir_name(g_head), l, f, r,
           dir_name(dl), l, dir_name(df), f, dir_name(dr), r);
#endif
}

/* Can the mouse drive from (x,y) towards d? Only through a side it has
 * SEEN to be open. */
static bool side_open(int x, int y, int d)
{
    return in_maze(x + dir_dx(d), y + dir_dy(d)) &&
           (g_known[x][y] & MM_WALL(d)) &&
           !(g_walls[x][y] & MM_WALL(d));
}

/* ==== 3. Moving and tracking the pose =================================== */
/* The HAL does not know where the mouse is. We change x, y and heading
 * ourselves, and only after the HAL says the move worked (MM_OK). */

static bool turn_to(int d)
{
    int q = ((d - g_head) % 4 + 4) % 4;          /* quarter turns to the right: 0..3 */
    if (q == 0) return true;
    if (mm_turn(q == 3 ? -1 : q) != MM_OK) {     /* 1 right, 2 around, -1 left */
        mm_log("task1: mm_turn failed at (%d,%d)", g_x, g_y);
        return false;
    }
    g_head = d;                                  /* heading changes only now */
    return true;
}

static bool forward_one(void)
{
    if (mm_forward(1) != MM_OK) {
        mm_log("task1: mm_forward blocked at (%d,%d) facing %c",
               g_x, g_y, dir_name(g_head));
        return false;
    }
    g_x += dir_dx(g_head);                       /* position changes only now */
    g_y += dir_dy(g_head);
    g_moves++;
    return true;
}

/* ==== 5. The map dump =================================================== */

static void report_map(void)
{
    int x, y;
    mm_map_begin();
    for (y = 0; y < g_h; y++)
        for (x = 0; x < g_w; x++)
            mm_map_cell(x, y, g_walls[x][y], g_known[x][y]);
    mm_map_end();
}

/* ==== Visiting every reachable cell ===================================== */

static bool explore_all(void)
{
    sense_cell();                                /* the start cell */

    for (;;) {
        int order[3], i, d = -1;

        /* an open neighbour not visited yet? try straight, right, left */
        order[0] = g_head;
        order[1] = dir_right(g_head);
        order[2] = dir_left(g_head);
        for (i = 0; i < 3 && d < 0; i++)
            if (side_open(g_x, g_y, order[i]) &&
                !g_visited[g_x + dir_dx(order[i])][g_y + dir_dy(order[i])])
                d = order[i];
        if (d < 0 && side_open(g_x, g_y, dir_back(g_head)) &&
            !g_visited[g_x + dir_dx(dir_back(g_head))][g_y + dir_dy(dir_back(g_head))])
            d = dir_back(g_head);

        if (d >= 0) {                            /* go forward into a new cell */
            if (!turn_to(d) || !forward_one()) return false;
            g_back[g_x][g_y] = (int8_t)dir_back(d);   /* remember the way back */
            sense_cell();
            continue;
        }

        if (g_back[g_x][g_y] < 0)                /* back at the start: all done */
            break;

        /* nothing new from here: drive one cell back the way we came */
        if (!turn_to(g_back[g_x][g_y]) || !forward_one()) return false;
    }

    /* If the mouse moved at all, it came back into (0,0) heading south or
     * west, so one more look sees the south wall of the start cell: the only
     * side of a visited cell that was never in view. */
    sense_cell();
    return true;
}

/* ==== Entry point ======================================================= */

void solver_run(void)
{
    int x, y, seen = 0;

    mm_begin(TEAM_ID);
    if (!maze_init()) return;

    map_init();
    g_x = 0; g_y = 0; g_head = MM_NORTH;         /* the start: (0,0) facing north */
    g_moves = 0;

    if (!explore_all())
        mm_log("task1: stopped early at (%d,%d); the dump shows what was seen", g_x, g_y);

    for (x = 0; x < g_w; x++)
        for (y = 0; y < g_h; y++)
            if (g_visited[x][y]) seen++;
    mm_log("task1: %d of %d cells visited, %d cells driven, now at (%d,%d) facing %c",
           seen, g_w * g_h, g_moves, g_x, g_y, dir_name(g_head));

    report_map();
}
