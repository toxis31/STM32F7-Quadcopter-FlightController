#ifndef BAROMETRE_H
#define BAROMETRE_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

// Variables globales lisibles par le main.c et le PID
extern float altitude_relative_m;
extern uint8_t baro_found;

#define LPS22HH_I2C_ADDR   (0x5D << 1)
#define LPS22HH_PRESS_OUT_XL 0x28 // Début des registres de données (Pression + Température)

void Baro_Init(void);
void Baro_Process_IT(uint8_t *raw_data);

#endif
