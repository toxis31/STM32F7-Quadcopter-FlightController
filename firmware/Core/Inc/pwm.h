/* USER CODE BEGIN Header */
#ifndef __ESC_PWM_H
#define __ESC_PWM_H

#include "main.h"

void ESC_Init_And_Calibrate(void);
void ESC_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

#endif
/* USER CODE END Header */
