#ifndef HC_SR04_H
#define HC_SR04_H

#include "main.h"  // Always include main.h for HAL libraries

// Function prototypes
void HC_SR04_Init(void);
void HC_SR04_Trigger(void);
uint16_t inline HC_SR04_Distance_Calculate(uint32_t pulse_us);

#endif /* OLED_DISPLAY_H */
