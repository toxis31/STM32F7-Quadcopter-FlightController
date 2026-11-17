/* USER CODE BEGIN Header */
#ifndef __MTF_H
#define __MTF_H

#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Variables globales lues par le main.c (pour le PID et la télémétrie)
extern uint32_t sensor_distance_mm;
extern uint32_t sensor_distance_filtered_mm;
extern int16_t  sensor_flow_x;
extern int16_t  sensor_flow_y;
extern uint8_t  sensor_flow_quality;
extern uint8_t  sensor_distance_status;

// Octet de réception UART pour les interruptions
extern uint8_t mtf_rx_byte;

// Fonctions publiques
void MTF_Init(void);
void MTF_Process_Byte(uint8_t data);

#endif
/* USER CODE END Header */
