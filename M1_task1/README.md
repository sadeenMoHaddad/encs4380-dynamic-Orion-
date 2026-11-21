# M1 Task 1: Map and position (Team T13)

The Task 1 solver maps every reachable cell, and its map dump matches the simulator maze exactly on the 4x4, 8x8 and 16x16 practice mazes: 0 wrong sides out of 64, 256 and 1,024. Full report with screenshots: [Micromouse_Task1_T13.pdf](Micromouse_Task1_T13.pdf).

## Requirements

| Task 1 requirement | Where it is met in `solver.c` |
| --- | --- |
| Each cell's walls in 4 bits, written with `MM_WALL(dir)`, no hard-coded numbers | `g_walls` and `g_known` (one `uint8_t` per cell); `map_init()`, `map_set_side()` |
| Every wall recorded on both cells that share it | `map_set_side()` writes the cell and the neighbour's opposite side, `(d + 2) % 4` |
| (x, y) and heading tracked after every move | `turn_to()`, `forward_one()` update the pose only after `MM_OK`; `sense_cell()` turns left/front/right into absolute walls |
| Maze size and goal read from `mm_maze()` | `maze_init()` |
| Map dump: `mm_map_begin()`, `mm_map_cell()` for each cell, `mm_map_end()` | `report_map()` |

To fill the map, the mouse walks the maze depth-first and comes back to the start, so every reachable cell is seen. After each new cell it also calls the flood fill from `M1_task2` and shows each cell's distance to the goal; this does not change the map.

## Files

| File | What it is |
| --- | --- |
| `solver.c` | The Task 1 solver |
| `dump4.txt` | Map dump from `practice4_1` (4x4) |
| `dump.txt` | Map dump from `practice8_offcentre` (8x8) |
| `dump16.txt` | Map dump from `practice16_1` (16x16) |
| `Micromouse_Task1_T13.pdf` | Task 1 report: implementation, screenshots and checks |

## Evidence: map dumps against the real mazes

| Dump | Maze | Cells | Sides known | Unknown | Wrong | CRC |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `dump4.txt` | practice4_1 (4x4) | 16 | 64 | 0 | 0 | `32F8` |
| `dump.txt` | practice8_offcentre (8x8) | 64 | 256 | 0 | 0 | `8106` |
| `dump16.txt` | practice16_1 (16x16) | 256 | 1,024 | 0 | 0 | `B35B` |

Each dump was copied from the mms Run Output and checked against the maze file side by side (`mm_check.py --dump`, see the report). Every side of every cell is known and correct. Task 1 maps the whole maze whatever the goal is, so these runs used the default centre goal.

## Build and run

This folder holds only our code. Put it together with the course kit's student folder:

1. Copy `solver.c` from this folder, and `flood_fill.c`, `flood_fill.h`, `flood_fill_display.c`, `flood_fill_display.h` from `M1_task2`, into the kit's student folder.
2. In mms set:

| mms field | Value |
| --- | --- |
| Directory | the kit's student folder |
| Build Command | `gcc -O2 -std=c99 -o solver.exe main_sim.c hal_sim.c mm_common.c solver.c flood_fill.c flood_fill_display.c` |
| Run Command | full path to `solver.exe` (Windows) or `./solver.exe` (Linux) |

3. Load a maze from `mazes/`, press Build, then Run. The Run Output ends with the `@MAP ... @END` dump.
