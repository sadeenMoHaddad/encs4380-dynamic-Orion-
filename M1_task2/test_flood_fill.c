
#include <stdio.h>
#include <string.h>
#include "flood_fill.h"

static uint8_t walls[FF_MAX_SIZE][FF_MAX_SIZE];
static uint8_t known[FF_MAX_SIZE][FF_MAX_SIZE];
static int dist[FF_MAX_SIZE][FF_MAX_SIZE];

static const int test_dx[4] = { 0, 1, 0, -1 };
static const int test_dy[4] = { 1, 0, -1, 0 };
static const int opposite[4] = { 2, 3, 0, 1 };

static void clear_map(void)
{
    memset(walls, 0, sizeof walls);
    memset(known, 0, sizeof known);
    memset(dist, 0, sizeof dist);
}

static void set_side(int width, int height,
                     int x, int y, int direction, int is_wall)
{
    int nx = x + test_dx[direction];
    int ny = y + test_dy[direction];
    uint8_t bit = FF_WALL(direction);
    uint8_t other_bit = FF_WALL(opposite[direction]);

    known[x][y] |= bit;

    if (is_wall)
        walls[x][y] |= bit;
    else
        walls[x][y] &= (uint8_t)~bit;

    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
        known[nx][ny] |= other_bit;

        if (is_wall)
            walls[nx][ny] |= other_bit;
        else
            walls[nx][ny] &= (uint8_t)~other_bit;
    }
}

static int fail(const char *message)
{
    printf("FAIL: %s\n", message);
    return 1;
}

/* Test 1: Known path and next direction. */
static int test_known_path(void)
{
    int x, y;

    clear_map();

    for (x = 0; x < 4; x++) {
        set_side(4, 4, x, 0, FF_SOUTH, 1);
        set_side(4, 4, x, 3, FF_NORTH, 1);
    }

    for (y = 0; y < 4; y++) {
        set_side(4, 4, 0, y, FF_WEST, 1);
        set_side(4, 4, 3, y, FF_EAST, 1);
    }

    for (x = 0; x < 4; x++) {
        for (y = 0; y < 4; y++) {
            if (x < 3)
                set_side(4, 4, x, y, FF_EAST, 0);

            if (y < 3)
                set_side(4, 4, x, y, FF_NORTH, 0);
        }
    }

    set_side(4, 4, 0, 1, FF_EAST, 1);

    if (!ff_compute(4, 4, walls, known, 1, 1, 2, 2, dist))
        return fail("known path computation failed");

    if (dist[1][1] != 0 || dist[2][2] != 0 ||
        dist[0][0] != 2)
        return fail("incorrect BFS distances");

    if (ff_next_direction(4, 4, walls, known, dist,
                          0, 0, FF_NORTH) != FF_EAST)
        return fail("incorrect next direction");

    return 0;
}

/* Test 2: Unknown passage policies. */
static int test_unknown_policy(void)
{
    clear_map();

    if (!ff_compute(2, 2, walls, known, 1, 1, 1, 1, dist))
        return fail("known BFS failed");

    if (dist[0][0] != -1)
        return fail("known BFS crossed an unknown passage");

    if (!ff_compute_explore(2, 2, walls, known, 1, 1, 1, 1, dist))
        return fail("exploration BFS failed");

    if (dist[0][0] != 2)
        return fail("exploration BFS did not cross potential passages");

    if (ff_next_direction_explore(2, 2, walls, known, dist,
                                  0, 0, FF_NORTH) != FF_NORTH)
        return fail("incorrect exploration direction");

    return 0;
}

/* Test 3: A wall reported by either cell blocks the passage. */
static int test_wall_on_either_side(void)
{
    clear_map();

    known[0][0] |= FF_WALL(FF_EAST);
    known[1][0] |= FF_WALL(FF_WEST);
    walls[1][0] |= FF_WALL(FF_WEST);

    if (!ff_compute_explore(2, 1, walls, known, 1, 0, 1, 0, dist))
        return fail("wall test computation failed");

    if (dist[0][0] != -1)
        return fail("crossed a wall reported by neighbor");

    return 0;
}

/* Test 4: A 1x1 maze where the start is already the goal. */
static int test_single_cell(void)
{
    clear_map();

    if (!ff_compute(1, 1, walls, known, 0, 0, 0, 0, dist))
        return fail("1x1 maze computation failed");

    if (dist[0][0] != 0)
        return fail("1x1 goal distance must be zero");

    if (ff_next_direction(1, 1, walls, known, dist,
                          0, 0, FF_NORTH) != -1)
        return fail("direction should not exist at the goal");

    return 0;
}

/* Test 5: A 2x2 goal region. */
static int test_goal_rectangle(void)
{
    clear_map();

    if (!ff_compute_explore(4, 4, walls, known,
                            1, 1, 2, 2, dist))
        return fail("rectangle goal computation failed");

    if (dist[1][1] != 0 || dist[1][2] != 0 ||
        dist[2][1] != 0 || dist[2][2] != 0)
        return fail("all goal cells must have distance zero");

    return 0;
}

/* Test 6: Unreachable cells remain -1. */
static int test_unreachable_cells(void)
{
    clear_map();

    /* Isolate cell (0,0) using walls on all four sides. */
    set_side(3, 3, 0, 0, FF_NORTH, 1);
    set_side(3, 3, 0, 0, FF_EAST, 1);
    set_side(3, 3, 0, 0, FF_SOUTH, 1);
    set_side(3, 3, 0, 0, FF_WEST, 1);

    if (!ff_compute(3, 3, walls, known, 2, 2, 2, 2, dist))
        return fail("unreachable test computation failed");

    if (dist[0][0] != -1)
        return fail("isolated cell should remain unreachable");

    return 0;
}

/* Test 7: Invalid heading. */
static int test_invalid_heading(void)
{
    clear_map();

    if (ff_next_direction(2, 2, walls, known, dist,
                          0, 0, 5) != -1)
        return fail("invalid heading was accepted");

    return 0;
}

/* Test 8: Invalid input. */
static int test_invalid_input(void)
{
    clear_map();

    if (ff_compute(0, 4, walls, known, 0, 0, 0, 0, dist))
        return fail("accepted zero width");

    if (ff_compute(4, 4, walls, known, -1, 0, 0, 0, dist))
        return fail("accepted negative goal coordinate");

    if (ff_compute(4, 4, walls, known, 0, 0, 4, 0, dist))
        return fail("accepted goal outside maze");

    return 0;
}


/* Test 9: Recalculate distances after discovering a new wall. */
static int test_new_wall_update(void)
{
    clear_map();

    /* Initially, all three cells are connected. */
    set_side(3, 1, 0, 0, FF_EAST, 0);
    set_side(3, 1, 1, 0, FF_EAST, 0);

    if (!ff_compute(3, 1, walls, known,
                    2, 0, 2, 0, dist))
        return fail("initial BFS computation failed");

    if (dist[0][0] != 2 || dist[1][0] != 1 ||
        dist[2][0] != 0)
        return fail("initial distances are incorrect");

    /* A new wall is discovered between cell 1 and the goal. */
    set_side(3, 1, 1, 0, FF_EAST, 1);

    /* Recalculate the distances using the updated map. */
    if (!ff_compute(3, 1, walls, known,
                    2, 0, 2, 0, dist))
        return fail("BFS recomputation failed");

    if (dist[0][0] != -1 || dist[1][0] != -1 ||
        dist[2][0] != 0)
        return fail("distances were not updated after new wall");

    return 0;
}


int main(void)
{
    if (test_known_path()) return 1;
    if (test_unknown_policy()) return 1;
    if (test_wall_on_either_side()) return 1;
    if (test_single_cell()) return 1;
    if (test_goal_rectangle()) return 1;
    if (test_unreachable_cells()) return 1;
    if (test_invalid_heading()) return 1;
    if (test_invalid_input()) return 1;
    if (test_new_wall_update()) return 1;

    puts("PASS: all 9 Flood Fill tests passed.");
    return 0;
}
