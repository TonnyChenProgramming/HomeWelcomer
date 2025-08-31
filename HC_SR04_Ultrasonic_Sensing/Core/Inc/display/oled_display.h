#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "main.h"  // Always include main.h for HAL libraries

// Function prototypes
void oled_init(void);

void oled_display_wavelength_and_distance(int wavelength, int distance);

#endif /* OLED_DISPLAY_H */
