#ifndef BOUSSOLE_H
#define BOUSSOLE_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

// Variable globale lisible par le main.c
extern volatile float actual_compass_heading;
extern uint8_t qmc_found;

// Constantes
#define QMC5883P_ADDR      (0x2C << 1)
#define QMC_P_DATA_X_LSB   0x01

// Prototypes des fonctions
void Compass_Init(void);
void Compass_Process_IT(uint8_t *raw_data);

#endif
