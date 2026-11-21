#ifndef FLOOD_FILL_DISPLAY_H
#define FLOOD_FILL_DISPLAY_H

#include "flood_fill.h"

void ff_display_distances(
    int width,
    int height,
    const int dist[FF_MAX_SIZE][FF_MAX_SIZE]);

#endif
