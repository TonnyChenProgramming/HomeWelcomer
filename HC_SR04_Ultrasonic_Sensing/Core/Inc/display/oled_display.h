#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "main.h"  // Always include main.h for HAL libraries

// Function prototypes
void oled_init(void);

void oled_display_wavelength_distance_rtc(int wavelength, int distance, uint32_t rtc);

#endif /* OLED_DISPLAY_H */

