#include "boussole.h"
#include <math.h>

// On importe le handle I2C configuré dans le main.c
extern I2C_HandleTypeDef hi2c2;

// Définitions
#define QMC_P_CHIP_ID      0x00
#define QMC_P_CTRL1        0x0A
#define QMC_P_CTRL2        0x0B
#define SMOOTH_STEPS       10

// --- Variables Globales ---
volatile float actual_compass_heading = 0.0f;
uint8_t qmc_found = 0;

// --- Calibration (Vos valeurs) ---
float offset_x = 12403.00f;
float offset_y = -4176.50f;
float offset_z = -7286.50f;
float scale_x = 1.01f;
float scale_y = 0.99f;
float scale_z = 0.99f;

// --- Buffers pour le lissage ---
float history_x[SMOOTH_STEPS] = {0};
float history_y[SMOOTH_STEPS] = {0};
float history_z[SMOOTH_STEPS] = {0};
uint8_t history_idx = 0;
uint8_t history_count = 0;

// =========================================================
// INITIALISATION (Appelée une seule fois dans le main)
// =========================================================
void Compass_Init(void) {
    uint8_t reg_val;
    uint8_t chip_id = 0;

    // Vérifier si la puce répond
    if (HAL_I2C_IsDeviceReady(&hi2c2, QMC5883P_ADDR, 3, 100) == HAL_OK) {
        HAL_I2C_Mem_Read(&hi2c2, QMC5883P_ADDR, QMC_P_CHIP_ID, 1, &chip_id, 1, 100);

        // Control Register 1 : Mode Continu, 200Hz, OSR 512
        reg_val = 0xFF;
        HAL_I2C_Mem_Write(&hi2c2, QMC5883P_ADDR, QMC_P_CTRL1, 1, &reg_val, 1, 100);

        // Control Register 2 : Plage de +/- 8 Gauss et Set/Reset ON
		reg_val = 0x09;
		HAL_I2C_Mem_Write(&hi2c2, QMC5883P_ADDR, QMC_P_CTRL2, 1, &reg_val, 1, 100);

        qmc_found = 1;
    } else {
        qmc_found = 0;
    }
}

void Compass_Process_IT(uint8_t *raw_data) {
    if (!qmc_found) return;

    // Reconstitution des données brutes
    int16_t mag_x = (int16_t)(raw_data[0] | (raw_data[1] << 8));
    int16_t mag_y = (int16_t)(raw_data[2] | (raw_data[3] << 8));
    int16_t mag_z = (int16_t)(raw_data[4] | (raw_data[5] << 8));

    // 1. Application de la Calibration
    float x_cal = ((float)mag_x - offset_x) * scale_x;
    float y_cal = ((float)mag_y - offset_y) * scale_y;
    float z_cal = ((float)mag_z - offset_z) * scale_z;

    // 2. Filtre de Lissage (Moyenne Mobile)
    history_x[history_idx] = x_cal;
    history_y[history_idx] = y_cal;
    history_z[history_idx] = z_cal;

    history_idx = (history_idx + 1) % SMOOTH_STEPS;
    if (history_count < SMOOTH_STEPS) history_count++;

    float sum_x = 0, sum_y = 0; // z n'est pas utilisé pour le cap basique plat
    for(int i = 0; i < history_count; i++) {
        sum_x += history_x[i];
        sum_y += history_y[i];
    }

    float x_smooth = sum_x / history_count;
    float y_smooth = sum_y / history_count;

    // 3. Calcul de l'azimut en degrés
    float azimut = atan2f(y_smooth, x_smooth) * (180.0f / M_PI) + 92.0f;
    if (azimut < 0.0f) {
        azimut += 360.0f;
    } else if (azimut >= 360.0f) {
        azimut -= 360.0f;
    }

    // Mise à jour de la variable globale
    actual_compass_heading = azimut;
}
