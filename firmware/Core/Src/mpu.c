/* USER CODE BEGIN Header */
#include "mpu.h"
#include <stdlib.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

#define MPU_ADDR (0x68 << 1)
#define RAD_TO_DEG 57.2957795f
#define SSF_GYRO  65.5f  // Sensibilité pour +/- 500 dps
#define PI 3.14159265359f

uint8_t rec_data[14];
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

float acc_total_vector = 0;
float ratio_y = 0;

// Valeurs de calibration de l'accéléromètre
int32_t acc_pitch_cal_value = 0;
int32_t acc_roll_cal_value = 0;

float gyro_raw[3] = {0,0,0};
float gyro_offset[3] = {0,0,0};
float gyro_angle[3] = {0,0,0};

float acc_raw[3] = {0,0,0};
float angle_pitch_acc = 0;
float angle_roll_acc = 0;

// --- NOUVEAU : Variables pour le Filtre Passe-Bas Logiciel ---
float filtered_gx = 0.0f;
float filtered_gy = 0.0f;
float filtered_gz = 0.0f;
// -------------------------------------------------------------

bool set_gyro_angles = false;

// Variables globales lues par main.c
float angle_pitch_output = 0;
float angle_roll_output = 0;
float rate_yaw_output = 0;

extern float roll_level_adjust;
extern float pitch_level_adjust;

void MPU_Init(void) {
    uint8_t data;

    // Wake up
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6B, 1, &data, 1, 100);

    // DLPF (Filtre passe-bas numérique MATÉRIEL)
	// 0x03 = 42Hz (Classique)
	// 0x04 = 20Hz (Très agressif contre les vibrations, parfait pour ton cas)
	data = 0x03;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1A, 1, &data, 1, 100);

    // Config Gyro FS_SEL=1 (500°/s)
    data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1B, 1, &data, 1, 100);

    // Config Accel AFS_SEL=2 (8g)
    data = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1C, 1, &data, 1, 100);
}

void Gyro_Calibrate(void) {
    int max_samples = 1000;
    gyro_offset[0] = 0;
    gyro_offset[1] = 0;
    gyro_offset[2] = 0;

    for (int i = 0; i < max_samples; i++) {
        MPU_Read_Sensor();
        gyro_offset[0] += gyro_raw[0];
        gyro_offset[1] += gyro_raw[1];
        gyro_offset[2] += gyro_raw[2];
        HAL_Delay(10);
    }

    gyro_offset[0] /= max_samples;
    gyro_offset[1] /= max_samples;
    gyro_offset[2] /= max_samples;
}

// Calibration de l'accéléromètre (Niveau)
void Acc_Calibrate(void) {

    for (int i = 0; i < 64; i++) {
        MPU_Read_Sensor();

        acc_roll_cal_value += acc_raw[0];
        acc_pitch_cal_value += acc_raw[1];

        HAL_Delay(4);
    }

    acc_pitch_cal_value = (int16_t)(acc_pitch_cal_value / 64);
    acc_roll_cal_value = (int16_t)(acc_roll_cal_value / 64);
}

void MPU_Read_Sensor(void) {
    if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, 1, rec_data, 14, 5) == HAL_OK) {

        Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data[1]);
        Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data[3]);
        Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data[5]);

        Gyro_X_RAW = (int16_t)(rec_data[8] << 8 | rec_data[9]);
        Gyro_Y_RAW = (int16_t)(rec_data[10] << 8 | rec_data[11]);
        Gyro_Z_RAW = (int16_t)(rec_data[12] << 8 | rec_data[13]);

        Gyro_Y_RAW = -Gyro_Y_RAW;
        Gyro_Z_RAW = -Gyro_Z_RAW;


        gyro_raw[0] = (float)Gyro_X_RAW;
        gyro_raw[1] = (float)Gyro_Y_RAW;
        gyro_raw[2] = (float)Gyro_Z_RAW;

        acc_raw[1] = (float)Accel_X_RAW;
        acc_raw[0] = (float)Accel_Y_RAW;
        acc_raw[2] = (float)Accel_Z_RAW;
    }
}

void MPU_Calculate_Angles(void) {
    float freq = 250.0f;

    // 1. Soustraire l'offset du Gyro (Valeurs brutes et bruitées)
    float raw_gx = gyro_raw[0] - gyro_offset[0];
    float raw_gy = gyro_raw[1] - gyro_offset[1];
    float raw_gz = gyro_raw[2] - gyro_offset[2];

    acc_raw[0] = acc_raw[0] - acc_roll_cal_value;
    acc_raw[1] = acc_raw[1] - acc_pitch_cal_value;

    // 2. --- TON FILTRE PASSE-BAS LOGICIEL (LPF) ---
    // 70% de la valeur précédente (mémoire) + 30% de la nouvelle valeur (réactivité)
    filtered_gx = 0.7f * filtered_gx + 0.3f * raw_gx / SSF_GYRO;
    filtered_gy = 0.7f * filtered_gy + 0.3f * raw_gy / SSF_GYRO;
    filtered_gz = 0.7f * filtered_gz + 0.3f * raw_gz / SSF_GYRO;

    // 3. On utilise les valeurs filtrées (propres) pour toute la suite de la physique !
    float gx = filtered_gx;
    float gy = filtered_gy;
    float gz = filtered_gz;

    // gy (Rotation sur l'axe Y) = Tangage (Pitch)
    // gx (Rotation sur l'axe X) = Roulis (Roll)
    gyro_angle[0] += gx / (freq);
    gyro_angle[1] += gy / (freq);
    gyro_angle[2] += gz / (freq); // Yaw

    // Transfert de lacet (Yaw transfer)
    float angle_z_rad = gz * (PI / (freq * 180.0f));
    gyro_angle[0] += gyro_angle[1] * sinf(angle_z_rad);
    gyro_angle[1] -= gyro_angle[0] * sinf(angle_z_rad);

    acc_total_vector = sqrtf(acc_raw[0]*acc_raw[0] + acc_raw[1]*acc_raw[1] + acc_raw[2]*acc_raw[2]);

    if(acc_total_vector != 0.0f) {

    	  if (fabsf(acc_raw[1]) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    	    angle_pitch_acc = asinf((float)acc_raw[1] / acc_total_vector) * 180/PI;              //Calculate the pitch angle.
    	  }
    	  if (fabsf(acc_raw[0]) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    	    angle_roll_acc = asinf((float)acc_raw[0] / acc_total_vector) * 180/PI;               //Calculate the roll angle.
    	  }

        // Filtre Complémentaire (Gyro propre + Accel pur)
        if (set_gyro_angles) {
        	gyro_angle[0] = gyro_angle[0] * 0.9996f + angle_roll_acc * 0.0004f;
        	gyro_angle[1] = gyro_angle[1] * 0.9996f + angle_pitch_acc  * 0.0004f;
        } else {
            gyro_angle[0] = angle_roll_acc;
            gyro_angle[1] = angle_pitch_acc;
            set_gyro_angles = true;
        }
    }

    // Enregistrement final pour le PID
    angle_pitch_output = gyro_angle[1];
    angle_roll_output  = gyro_angle[0];

    roll_level_adjust = angle_roll_output*15.0f;
    pitch_level_adjust = angle_pitch_output*15.0f;
}
/* USER CODE END Header */
