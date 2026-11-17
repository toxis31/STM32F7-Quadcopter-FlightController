#include "barometre.h"
#include <math.h>

extern I2C_HandleTypeDef hi2c2;

uint8_t baro_found = 0;
uint8_t baro_is_calibrated = 0;
float altitude_zero_m = 0.0f;
float altitude_absolue_m = 0.0f;
float altitude_relative_m = 0.0f;

void Baro_Init(void) {
    uint8_t baro_id = 0;

    // 1. Vérification du WHO_AM_I
    if (HAL_I2C_Mem_Read(&hi2c2, LPS22HH_I2C_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT, &baro_id, 1, 100) == HAL_OK) {
        if (baro_id == 0xB3) {
            // 2. Configuration (CTRL_REG1) : 75 Hz, Continuous Mode
            uint8_t ctrl_reg1_val = 0x5A;
            HAL_I2C_Mem_Write(&hi2c2, LPS22HH_I2C_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1_val, 1, 100);
            baro_found = 1;
        } else {
            baro_found = 0;
        }
    } else {
        baro_found = 0;
    }
}

void Baro_Process_IT(uint8_t *raw_data) {
    if (!baro_found) return;

    // --- TRAITEMENT DE LA PRESSION ---
    int32_t pressure_lsb = (int32_t)((uint32_t)raw_data[2] << 16 |
                                     (uint32_t)raw_data[1] << 8  |
                                     (uint32_t)raw_data[0]);

    if (pressure_lsb & 0x800000) pressure_lsb |= 0xFF000000;
    float pressure_hPa = (float)pressure_lsb / 4096.0f;

    // --- TRAITEMENT DE LA TEMPÉRATURE ---
    int16_t temp_lsb = (int16_t)((uint16_t)raw_data[4] << 8 | (uint16_t)raw_data[3]);
    float temperature_c = (float)temp_lsb / 100.0f;

    // --- CALCUL DE L'ALTITUDE ---
    float coefficient_temp = (temperature_c + 273.15f) / 0.0065f;
    altitude_absolue_m = coefficient_temp * (1.0f - powf(pressure_hPa / 1013.25f, 0.1903f));

    // Calibration du Zéro au démarrage
	if (baro_is_calibrated == 0) {
		altitude_zero_m = altitude_absolue_m;
		baro_is_calibrated = 1;
	}

	// Altitude brute par rapport au sol
	float altitude_relative_brute = altitude_absolue_m - altitude_zero_m;

	// --- LE FILTRE PASSE-BAS ---
	// Si c'est la toute première lecture, on initialise le filtre directement sur la bonne valeur
	static uint8_t filter_initialized = 0;
	if (filter_initialized == 0) {
		altitude_relative_m = altitude_relative_brute;
		filter_initialized = 1;
	} else {
		// Formule : 95% de l'ancienne valeur + 5% de la nouvelle
		altitude_relative_m = (altitude_relative_m * 0.95f) + (altitude_relative_brute * 0.05f);
	}
}
