#ifndef GPS_H
#define GPS_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

// --- Variables lues par le main.c ---
extern volatile int32_t lat_gps_actual;
extern volatile int32_t lon_gps_actual;
extern volatile uint8_t number_used_sats;
extern volatile uint8_t fix_type;
extern volatile uint8_t new_gps_data_available;
extern int32_t l_lat_gps;
extern int32_t l_lon_gps;

// --- Fonctions publiques ---
void GPS_Init(void);
void GPS_Process_Byte(uint8_t c);
void GPS_Interpolate(void);

#endif
