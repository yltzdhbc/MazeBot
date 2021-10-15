#ifndef BSP_LED_H
#define BSP_LED_H
#include "main.h"

typedef enum
{
    RED = 0,
    GREEN = 1,
    BLUE = 2,
    YELLOW = 3,
    MAGENTA = 4,
    CYAN = 5,
    WHITE = 6,
    BLACK = 7
} ledRgb_e;

extern ledRgb_e aRGB;

void LED_GRB_SHOW(ledRgb_e aRGB);
#endif
