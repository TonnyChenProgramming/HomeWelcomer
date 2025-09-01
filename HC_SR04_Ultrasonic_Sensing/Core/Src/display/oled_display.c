#include "oled_display.h"  // Include corresponding .h file
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdio.h"
#include "main.h"


// OLED Initialization Function
void oled_init(void) {
    ssd1306_Init();
}


void oled_display_wavelength_distance_rtc(int wavelength, int distance, uint32_t rtc)
{
    char buffer[30];
    ssd1306_Fill(Black);

    ssd1306_SetCursor(2, 2);
    sprintf(buffer, "wavelength: %d microseconds", wavelength);  // use %d for int
    ssd1306_WriteString(buffer, Font_6x8, White);

    ssd1306_SetCursor(2, 12);
    sprintf(buffer, "distance: %d cm", distance);      // use %% to print literal '%'
    ssd1306_WriteString(buffer, Font_6x8, White);

    ssd1306_SetCursor(2, 22);
    sprintf(buffer, "rtc: %ld ", rtc);      // use %% to print literal '%'
    ssd1306_WriteString(buffer, Font_6x8, White);

    ssd1306_UpdateScreen();
}

