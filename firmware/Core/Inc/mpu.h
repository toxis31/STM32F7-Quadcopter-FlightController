/* USER CODE BEGIN Header */
#ifndef __MPU_H
#define __MPU_H

#include "stm32f7xx_hal.h"
#include <stdbool.h>

// Variables que le main.c va utiliser pour les PID
extern float angle_pitch_output;
extern float angle_roll_output;
extern float rate_yaw_output; // IMPORTANT: On utilise la vitesse pour le lacet !

void MPU_Init(void);
void Gyro_Calibrate(void);
void Acc_Calibrate(void);
void MPU_Read_Sensor(void);
void MPU_Calculate_Angles(void);

#endif
/* USER CODE END Header */
