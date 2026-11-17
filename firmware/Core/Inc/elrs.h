/* USER CODE BEGIN Header */
#ifndef __ELRS_H
#define __ELRS_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define CRSF_FAILSAFE_TIMEOUT_MS 200

extern uint16_t rc_channels[16];
extern uint8_t failsafe_active;
extern bool is_armed;
extern uint8_t elrs_rx_byte;

void ELRS_Init(void);
void ELRS_Process_Byte(uint8_t byte);
void ELRS_Update_Failsafe(void);

#endif
/* USER CODE END Header */
