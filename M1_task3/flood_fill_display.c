#include <stdio.h>
#include "flood_fill_display.h"
#include "mouse_hal.h"

void ff_display_distances(
    int width,
    int height,
    const int dist[FF_MAX_SIZE][FF_MAX_SIZE])
{
    int x, y;
    char text[11];

    if (dist == NULL ||
        width < 1 || width > FF_MAX_SIZE ||
        height < 1 || height > FF_MAX_SIZE) {
        return;
    }

    for (x = 0; x < width; x++) {
        for (y = 0; y < height; y++) {
            if (dist[x][y] < 0) {
                text[0] = '-';   /* No route / unreachable */
                text[1] = '\0';
            } else {
                snprintf(text, sizeof(text), "%d", dist[x][y]);
            }

            mm_label(x, y, text);
        }
    }
}
