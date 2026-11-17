/* USER CODE BEGIN Header */
#include "pwm.h"

extern TIM_HandleTypeDef htim8;

void ESC_Init_And_Calibrate(void) {
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    HAL_Delay(1000);

//seq calibration
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 2000);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 2000);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 2000);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 2000);

    HAL_Delay(1000);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1000);

    HAL_Delay(1000);
}

void ESC_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
	// Routage vers le bon fil matériel en fonction des esc :
	    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, m1); // CH2 = Avant Gauche
	    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, m2); // CH3 = Avant Droit
	    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, m3); // CH4 = Arrière Droit
	    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, m4); // CH1 = Arrière Gauche
}
/* USER CODE END Header */
