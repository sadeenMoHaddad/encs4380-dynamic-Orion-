#include "mouse_hal.h"
#include "flood_fill.h"
#include "flood_fill_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define TEAM_ID             "T13"
#define LOG_EVERY_CELL      1
#define REFINE_PATH         1
#define REFINE_MAX_PASSES   64

static int g_w, g_h;
static int g_gx0, g_gy0, g_gx1, g_gy1;

static uint8_t g_walls[MM_MAX_SIZE][MM_MAX_SIZE];
static uint8_t g_known[MM_MAX_SIZE][MM_MAX_SIZE];
static int     g_dist[MM_MAX_SIZE][MM_MAX_SIZE];
static bool    g_visited[MM_MAX_SIZE][MM_MAX_SIZE];

static int g_x, g_y;
static int g_head;
static int g_moves;

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

static bool in_rect(int x, int y, int x0, int y0, int x1, int y1)
{
    return x >= x0 && x <= x1 && y >= y0 && y <= y1;
}

static bool in_goal(int x, int y)
{
    return in_rect(x, y, g_gx0, g_gy0, g_gx1, g_gy1);
}

static bool maze_init(void)
{
    mm_maze_t m = mm_maze();

    g_w = m.width;
    g_h = m.height;
    g_gx0 = m.goal_x0; g_gy0 = m.goal_y0;
    g_gx1 = m.goal_x1; g_gy1 = m.goal_y1;

    if (g_w < 1 || g_h < 1 || g_w > MM_MAX_SIZE || g_h > MM_MAX_SIZE) {
        mm_log("task3: maze %dx%d is not supported", g_w, g_h);
        return false;
    }
    mm_log("task3: maze %dx%d, goal (%d,%d)-(%d,%d)",
           g_w, g_h, g_gx0, g_gy0, g_gx1, g_gy1);
    return true;
}



static void map_set_side(int x, int y, int d, bool wall)
{
    int nx = x + dir_dx(d), ny = y + dir_dy(d);

    g_known[x][y] |= MM_WALL(d);
    if (wall) g_walls[x][y] |= MM_WALL(d);
    else      g_walls[x][y] &= (uint8_t)~MM_WALL(d);

    if (in_maze(nx, ny)) {
        int back = dir_back(d);
        g_known[nx][ny] |= MM_WALL(back);
        if (wall) g_walls[nx][ny] |= MM_WALL(back);
        else      g_walls[nx][ny] &= (uint8_t)~MM_WALL(back);
    }

    if (wall) mm_show_wall(x, y, d);
}

static void sense_cell(void)
{
    uint8_t s = mm_sense();
    bool l = (s & MM_SEE_LEFT)  != 0;
    bool f = (s & MM_SEE_FRONT) != 0;
    bool r = (s & MM_SEE_RIGHT) != 0;
    int dl = dir_left(g_head), df = g_head, dr = dir_right(g_head);

    map_set_side(g_x, g_y, dl, l);
    map_set_side(g_x, g_y, df, f);
    map_set_side(g_x, g_y, dr, r);
    g_visited[g_x][g_y] = true;
#if LOG_EVERY_CELL
    mm_log("(%d,%d) facing %c: L=%d F=%d R=%d  ->  %c=%d %c=%d %c=%d",
           g_x, g_y, dir_name(g_head), l, f, r,
           dir_name(dl), l, dir_name(df), f, dir_name(dr), r);
#endif
}

static bool turn_to(int d)
{
    int q = ((d - g_head) % 4 + 4) % 4;
    if (q == 0) return true;
    if (mm_turn(q == 3 ? -1 : q) != MM_OK) {
        mm_log("task3: mm_turn failed at (%d,%d)", g_x, g_y);
        return false;
    }
    g_head = d;
    return true;
}

static bool forward_one(void)
{
    if (mm_forward(1) != MM_OK) {
        mm_log("task3: mm_forward blocked at (%d,%d) facing %c",
               g_x, g_y, dir_name(g_head));
        return false;
    }
    g_x += dir_dx(g_head);
    g_y += dir_dy(g_head);
    g_moves++;
    return true;
}

static void report_map(void)
{
    int x, y;
    mm_map_begin();
    for (y = 0; y < g_h; y++)
        for (x = 0; x < g_w; x++)
            mm_map_cell(x, y, g_walls[x][y], g_known[x][y]);
    mm_map_end();
}

static bool step_towards(int tx0, int ty0, int tx1, int ty1, bool explore)
{
    int d;

    if (explore)
        ff_compute_explore(g_w, g_h, g_walls, g_known,
                           tx0, ty0, tx1, ty1, g_dist);
    else
        ff_compute(g_w, g_h, g_walls, g_known,
                   tx0, ty0, tx1, ty1, g_dist);
    ff_display_distances(g_w, g_h, g_dist);

    d = explore
        ? ff_next_direction_explore(g_w, g_h, g_walls, g_known,
                                    g_dist, g_x, g_y, g_head)
        : ff_next_direction(g_w, g_h, g_walls, g_known,
                            g_dist, g_x, g_y, g_head);
    if (d < 0) return false;

    return turn_to(d) && forward_one();
}

static bool go_exploring(int tx0, int ty0, int tx1, int ty1)
{
    while (!in_rect(g_x, g_y, tx0, ty0, tx1, ty1)) {
        if (!step_towards(tx0, ty0, tx1, ty1, true)) {
            mm_log("task3: no route to (%d,%d) from (%d,%d)",
                   tx0, ty0, g_x, g_y);
            return false;
        }
        sense_cell();
    }
    return true;
}

static bool refine_path(void)
{
    int pass;

    for (pass = 0; pass < REFINE_MAX_PASSES; pass++) {
        int d_opt, d_conf;

        ff_compute_explore(g_w, g_h, g_walls, g_known,
                           g_gx0, g_gy0, g_gx1, g_gy1, g_dist);
        d_opt = g_dist[0][0];
        ff_compute(g_w, g_h, g_walls, g_known,
                   g_gx0, g_gy0, g_gx1, g_gy1, g_dist);
        d_conf = g_dist[0][0];

        mm_log("task3: refine pass %d: optimistic=%d confirmed=%d",
               pass, d_opt, d_conf);

        if (d_conf >= 0 && d_conf == d_opt)
            return true;

        if (!go_exploring(g_gx0, g_gy0, g_gx1, g_gy1)) return false;
        if (!go_exploring(0, 0, 0, 0))                 return false;
    }
   mm_log("task3: refinement did not converge");
return false;
}

static bool speed_run(void)
{
    while (!in_goal(g_x, g_y)) {
        if (!step_towards(g_gx0, g_gy0, g_gx1, g_gy1, false)) {
            mm_log("task3: speed run blocked at (%d,%d)", g_x, g_y);
            return false;
        }
    }
    return true;
}

static bool run_all_phases(void)
{
    int start_moves, run_moves;

    sense_cell();

    if (!go_exploring(g_gx0, g_gy0, g_gx1, g_gy1)) return false;
    mm_log("task3: goal reached after %d cells", g_moves);

    if (!go_exploring(0, 0, 0, 0)) return false;
    mm_log("task3: back at start after %d cells", g_moves);

#if REFINE_PATH
    if (!refine_path()) return false;
#endif

    start_moves = g_moves;
    if (!speed_run()) return false;
    run_moves = g_moves - start_moves;
    mm_log("task3: speed run done in %d cells (total driven %d)",
           run_moves, g_moves);
    return true;
}

void solver_run(void)
{
    int x, y, seen = 0;

    mm_begin(TEAM_ID);
    if (!maze_init()) return;

    map_init();
    g_x = 0; g_y = 0; g_head = MM_NORTH;
    g_moves = 0;

    if (!run_all_phases())
        mm_log("task3: stopped early at (%d,%d); the dump shows what was seen",
               g_x, g_y);

    for (x = 0; x < g_w; x++)
        for (y = 0; y < g_h; y++)
            if (g_visited[x][y]) seen++;
    mm_log("task3: %d of %d cells visited, %d cells driven, now at (%d,%d) facing %c",
           seen, g_w * g_h, g_moves, g_x, g_y, dir_name(g_head));

    report_map();
}
