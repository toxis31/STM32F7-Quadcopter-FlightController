/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Clean - No Printf - 250Hz Loop)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "pwm.h"
#include "mpu.h"
#include "elrs.h"
#include "mtf.h"
#include "boussole.h"
#include "barometre.h"
#include "gps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_PITCH_KP  1.3f // 1.3
#define PID_PITCH_KI  0.025f // 0.04
#define PID_PITCH_KD  18.0f // 18

#define PID_ROLL_KP   1.3f
#define PID_ROLL_KI   0.025f
#define PID_ROLL_KD   18.0f

#define PID_YAW_KP    4.0f
#define PID_YAW_KI    0.02f
#define PID_YAW_KD    0.0f

int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-).
int pid_max_pitch = 400;                    //Maximum output of the PID-controller (+/-).
int pid_max_yaw = 400;                    //Maximum output of the PID-controller (+/-).

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float pid_error_temp = 0;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_roll_setpoint = 0;
float pid_pitch_setpoint = 0;
float pid_yaw_setpoint = 0;

float roll_level_adjust = 0;
float pitch_level_adjust = 0;

uint16_t rc_throttle = 1000;
uint16_t esc_1, esc_2, esc_3, esc_4;

extern float filtered_gx;
extern float filtered_gy;
extern float filtered_gz;

extern uint8_t elrs_rx_byte;

extern int32_t acc_pitch_cal_value;
extern int32_t acc_roll_cal_value;

// PID altitude
float pid_altitude_setpoint = 0;
float pid_i_mem_altitude = 0;
float pid_last_altitude_d_error = 0;
float pid_output_altitude = 0;
float pid_error_altitude = 0;

float pid_p_gain_altitude = 0.2; // 0.2
float pid_i_gain_altitude = 0.0001; // 0.0001
float pid_d_gain_altitude = 5.0; // 5
int pid_max_altitude = 150; // Limite l'action sur les gaz

// Le point de stationnaire (Hover).
int hover_throttle;

bool alt_hold_active = false;
bool previous_alt_hold_state = false;

// Variables pour le séquenceur I2C
uint8_t sequencer_counter = 0;
uint8_t i2c_active_sensor = 0; // 1 = Boussole, 2 = Baromètre

//boussole qmc5883p
uint8_t qmc_raw_data[6];

//baro lps22hh
uint8_t baro_raw_data[5];

//gps m10
extern uint8_t gps_rx_byte;

// --- Variables PID GPS ---
float gps_p_gain = 1.0f; //2.7
float gps_d_gain = 5.0f; //6.5

int32_t l_lat_waypoint = 0;
int32_t l_lon_waypoint = 0;

int32_t gps_lat_error = 0;
int32_t gps_lon_error = 0;
int32_t gps_lat_error_previous = 0;
int32_t gps_lon_error_previous = 0;

float gps_pitch_adjust_north = 0.0f;
float gps_roll_adjust_north = 0.0f;
float gps_pitch_adjust = 0.0f;
float gps_roll_adjust = 0.0f;

bool gps_hold_active = false;
bool previous_gps_hold_state = false;

// isen : 50.634096, 3.048933
// palais rameau 50.634472, 3.049654
// 10m nord : 50.63497640045477, 3.0488501656479223
int32_t target_lat = 506342430;
int32_t target_lon = 30491792;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Calculate_PID();
void Apply_Motor_Speed(uint16_t throttle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Calculate_PID() {
	pid_roll_setpoint = 0;

	if (rc_channels[3] > 1508)pid_roll_setpoint = rc_channels[3] - 1508;
	else if (rc_channels[3] < 1492)pid_roll_setpoint = rc_channels[3] - 1492;

	// Le GPS incline le drone en roll
	pid_roll_setpoint += gps_roll_adjust;

	pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
	pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

	pid_pitch_setpoint = 0;
	if (rc_channels[1] > 1508)pid_pitch_setpoint = rc_channels[1] - 1508;
	else if (rc_channels[1] < 1492)pid_pitch_setpoint = rc_channels[1] - 1492;

	//Le GPS incline le drone en pitch
	pid_pitch_setpoint -= gps_pitch_adjust;

	pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
	pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

	pid_yaw_setpoint = 0;
	//We need a little dead band of 16us for better results.
	if (rc_channels[2] > 1050) { //Do not yaw when turning off the motors.
	if (rc_channels[0] > 1508)pid_yaw_setpoint = (rc_channels[0] - 1508) / 3.0;
	else if (rc_channels[0] < 1492)pid_yaw_setpoint = (rc_channels[0] - 1492) / 3.0;
	}

	//Roll calculations
	pid_error_temp = filtered_gx - pid_roll_setpoint;
	pid_i_mem_roll += PID_ROLL_KI * pid_error_temp;
	if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
	else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = PID_ROLL_KP * pid_error_temp + pid_i_mem_roll + PID_ROLL_KD * (pid_error_temp - pid_last_roll_d_error);
	if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
	else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;

	//Pitch calculations
	pid_error_temp = filtered_gy - pid_pitch_setpoint;
	pid_i_mem_pitch += PID_PITCH_KI * pid_error_temp;
	if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
	else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = PID_PITCH_KP * pid_error_temp + pid_i_mem_pitch + PID_PITCH_KD * (pid_error_temp - pid_last_pitch_d_error);
	if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
	else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = filtered_gz - pid_yaw_setpoint;
	pid_i_mem_yaw += PID_YAW_KI * pid_error_temp;
	if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
	else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = PID_YAW_KP * pid_error_temp + pid_i_mem_yaw + PID_YAW_KD * (pid_error_temp - pid_last_yaw_d_error);
	if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
	else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;

}

void Apply_Motor_Speed(uint16_t throttle) {

	esc_1 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;
	esc_2 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
	esc_3 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
	esc_4 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;

    if (esc_1 < 1000) esc_1 = 1000;
    if (esc_1 > 2000) esc_1 = 2000;

    if (esc_2 < 1000) esc_2 = 1000;
    if (esc_2 > 2000) esc_2 = 2000;

    if (esc_3 < 1000) esc_3 = 1000;
    if (esc_3 > 2000) esc_3 = 2000;

    if (esc_4 < 1000) esc_4 = 1000;
    if (esc_4 > 2000) esc_4 = 2000;


    ESC_Write(esc_1, esc_2, esc_3, esc_4);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  MPU_Init();
  HAL_Delay(200);
  Gyro_Calibrate();
  Acc_Calibrate();
  HAL_Delay(200);
  MTF_Init();
  Compass_Init();
  Baro_Init();
  GPS_Init();

  // Le drone dérive vers l'Arrière-Droit.
  // On diminue les valeurs de calibration pour générer une fausse
  // inclinaison positive, forçant le drone à compenser vers l'Avant-Gauche.

  acc_pitch_cal_value -= 15; // Compense la dérive arrière
  acc_roll_cal_value  -= 25; // Compense la dérive droite

  HAL_TIM_Base_Start_IT(&htim3);
  ELRS_Init();
  HAL_Delay(2000);
  ESC_Init_And_Calibrate(); ////////////////

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20404768;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 107;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 215;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2499;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 416666;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {

        // 1. Gestion Failsafe et Armement
        ELRS_Update_Failsafe();

        if (!failsafe_active && rc_channels[4] > 1700) {
            is_armed = true;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        } else {
            is_armed = false;
            ESC_Write(1000,1000,1000,1000);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        }

        // ---   I2C ---
		sequencer_counter++;
		if (sequencer_counter >= 5) {
			sequencer_counter = 0;
		}
		// Créneau de la Boussole (T = 4 ms)
		if (sequencer_counter == 1 && qmc_found) {
			i2c_active_sensor = 1; // <-- ON NOTE QUE LA BOUSSOLE PARLE
			HAL_I2C_Mem_Read_IT(&hi2c2, QMC5883P_ADDR, QMC_P_DATA_X_LSB, 1, qmc_raw_data, 6);
		}

		// Créneau du Baromètre (T = 12 ms)
		else if (sequencer_counter == 3 && baro_found) {
			i2c_active_sensor = 2; // <-- ON NOTE QUE LE BAROMÈTRE PARLE
			HAL_I2C_Mem_Read_IT(&hi2c2, LPS22HH_I2C_ADDR, LPS22HH_PRESS_OUT_XL, 1, baro_raw_data, 5);
		}

		// Interpolation douce du GPS (Lissage 5Hz -> 250Hz)
		GPS_Interpolate();

        // 2. Lecture MPU
        MPU_Read_Sensor();
        MPU_Calculate_Angles();

        //is_armed=true; ////////////////////

        // 3. Calculs PID
		if (is_armed) {

			rc_throttle = rc_channels[2];

			/*
			rc_channels[2]=1150; /////////////////////
			rc_channels[5]=2000; /////////////////////
			*/

			alt_hold_active = (sensor_distance_status == 1 && rc_channels[5] > 1700 && rc_channels[2] > 1100); /////1000

			// Détection activation alt_hold
			if (alt_hold_active && !previous_alt_hold_state) {
				hover_throttle = rc_channels[2];
				pid_altitude_setpoint = (float)sensor_distance_mm;
				pid_i_mem_altitude = 0;
				pid_last_altitude_d_error = 0;
			}

			// Gestion altitude hold
	        if (alt_hold_active) {

	        	// 1. GESTION DE LA CONSIGNE PAR LE PILOTE (Logique Brokking)
	        	// 1450

				if (rc_throttle > 1600) {
					// Le pilote veut monter : on augmente la consigne d'altitude
					pid_altitude_setpoint += (rc_throttle - 1600) * 0.02f;
				}
				else if (rc_throttle < 1300) {
					// Le pilote veut descendre : on baisse la consigne
					pid_altitude_setpoint -= (1300 - rc_throttle) * 0.02f;
				}


				// Sécurité : On empêche de demander une altitude sous 10cm ou au-dessus de 2m
				if (pid_altitude_setpoint < 200) pid_altitude_setpoint = 200;
				if (pid_altitude_setpoint > 3000) pid_altitude_setpoint = 3000;


				// 2. CALCUL DU PID D'ALTITUDE
				// Erreur en millimètres = Consigne - Réalité
				pid_error_altitude = pid_altitude_setpoint - (float)sensor_distance_mm;

				pid_i_mem_altitude += (pid_i_gain_altitude * pid_error_altitude);
				if (pid_i_mem_altitude > pid_max_altitude) pid_i_mem_altitude = pid_max_altitude;
				else if (pid_i_mem_altitude < pid_max_altitude * -1) pid_i_mem_altitude = pid_max_altitude * -1;

				float derivative_alt = pid_error_altitude - pid_last_altitude_d_error;

				pid_output_altitude = (pid_p_gain_altitude * pid_error_altitude) + pid_i_mem_altitude + (pid_d_gain_altitude * derivative_alt);

				// Limitation de la sortie pour ne pas envoyer les moteurs dans l'espace
				if (pid_output_altitude > pid_max_altitude) pid_output_altitude = pid_max_altitude;
				else if (pid_output_altitude < pid_max_altitude * -1) pid_output_altitude = pid_max_altitude * -1;

				pid_last_altitude_d_error = pid_error_altitude;

				// 3. APPLICATION AUX GAZ
				// Au lieu de prendre le rc_throttle pur, on prend le point de stationnaire + le PID
				rc_throttle = hover_throttle + (int16_t)pid_output_altitude;

	        } else {
	            // LE MAINTIEN D'ALTITUDE EST DÉSACTIVÉ (Ou le capteur ne voit plus le sol)
	            // On réinitialise les variables pour que le système soit prêt pour la prochaine activation
	            pid_altitude_setpoint = (float)sensor_distance_mm; // La consigne devient l'altitude actuelle
	            pid_i_mem_altitude = 0;
	            pid_last_altitude_d_error = 0;
	            pid_output_altitude = 0;
	        }

	        // ========================================================
			// MAINTIEN DE POSITION GPS (POSITION HOLD)
			// ========================================================

			// On active si l'interrupteur est ON (ex: canal 7) ET si le GPS a un bon signal (Fix 3D et + de 7 sats)
			gps_hold_active = (rc_channels[6] > 1700 && fix_type == 3 && number_used_sats >= 3);

			if (gps_hold_active && !previous_gps_hold_state) {
			    // Au lieu de l_lat_waypoint = l_lat_gps, on force la cible
			    l_lat_waypoint = target_lat;
			    l_lon_waypoint = target_lon;

			    // Verrouillage de l'altitude à 2000mm (2 mètres)
			    pid_altitude_setpoint = 2000.0f;

			    gps_lat_error_previous = 0;
			    gps_lon_error_previous = 0;
			}

			if (gps_hold_active) {

				// 1. Calcul de l'erreur géographique (Cible - Actuelle)
				gps_lat_error = l_lat_waypoint - l_lat_gps;
				gps_lon_error = l_lon_waypoint - l_lon_gps;

				// Calcul de la dérivée (vitesse de dérive)
				int32_t gps_lat_derivative = gps_lat_error - gps_lat_error_previous;
				int32_t gps_lon_derivative = gps_lon_error - gps_lon_error_previous;

				gps_lat_error_previous = gps_lat_error;
				gps_lon_error_previous = gps_lon_error;

				// 2. Calcul de la force de correction globale (Nord/Est) avec un PD Controller
				gps_pitch_adjust_north = ((float)gps_lat_error * gps_p_gain) + ((float)gps_lat_derivative * gps_d_gain);
				gps_roll_adjust_north  = ((float)gps_lon_error * gps_p_gain) + ((float)gps_lon_derivative * gps_d_gain);

				// 3. LA MATRICE DE ROTATION (Conversion Global -> Local)
				// On convertit le cap de la boussole en radians
				float heading_rad = actual_compass_heading * (M_PI / 180.0f);

				// On fait tourner les vecteurs
				gps_pitch_adjust = (gps_pitch_adjust_north * cosf(heading_rad)) + (gps_roll_adjust_north * sinf(heading_rad));
				gps_roll_adjust  = (gps_roll_adjust_north * cosf(heading_rad))  - (gps_pitch_adjust_north * sinf(heading_rad));

				// 4. Limites de sécurité
				// 300 correspond à environ 10 degrés d'inclinaison selon vos réglages PID
				if (gps_pitch_adjust > 150.0f) gps_pitch_adjust = 150.0f;
				if (gps_pitch_adjust < -150.0f) gps_pitch_adjust = -150.0f;
				if (gps_roll_adjust > 150.0f) gps_roll_adjust = 150.0f;
				if (gps_roll_adjust < -150.0f) gps_roll_adjust = -150.0f;

			} else {
				// Si le GPS Hold est désactivé, on coupe les corrections
				gps_pitch_adjust = 0.0f;
				gps_roll_adjust = 0.0f;
			}

			previous_gps_hold_state = gps_hold_active;
	        previous_alt_hold_state = alt_hold_active;

			Calculate_PID();

			Apply_Motor_Speed(rc_throttle);

		} else {
			// Drone désarmé
			rc_throttle = 1000;
			pid_i_mem_pitch = 0;
			pid_i_mem_roll = 0;
			pid_i_mem_yaw = 0;
			pid_last_pitch_d_error = 0;
			pid_last_roll_d_error = 0;
			pid_last_yaw_d_error = 0;

			ESC_Write(1000,1000,1000,1000);
		}

		// -------------------------------------------------------------
		// SÉCURITÉ AU SOL (Anti-Windup)
		// Si les gaz sont au minimum (le drone est posé), on force
		// la mémoire de l'intégrale à zéro pour l'empêcher de s'emballer.
		// -------------------------------------------------------------
		if (rc_throttle < 1100) {
			pid_i_mem_roll = 0;
			pid_i_mem_pitch = 0;
			pid_i_mem_yaw = 0;
		}
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//manette
    if (huart->Instance == UART4) {
        ELRS_Process_Byte(elrs_rx_byte);
        HAL_UART_Receive_IT(&huart4, &elrs_rx_byte, 1);
    }
    //mtf-02p
    else if(huart->Instance == UART5) {
		MTF_Process_Byte(mtf_rx_byte);
		HAL_UART_Receive_IT(&huart5, &mtf_rx_byte, 1);
	}
    //gps
    else if(huart->Instance == USART2) {
		GPS_Process_Byte(gps_rx_byte);
		HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(&huart4, &elrs_rx_byte, 1);
    }
    else if (huart->Instance == UART5) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		HAL_UART_Receive_IT(&huart5, &mtf_rx_byte, 1);
	}
    else if (huart->Instance == USART2) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);
		HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C2) {

        if (i2c_active_sensor == 1) {
            Compass_Process_IT(qmc_raw_data);
        }
        else if (i2c_active_sensor == 2) {
            Baro_Process_IT(baro_raw_data);
        }

        // On remet à zéro pour éviter les faux déclenchements
        i2c_active_sensor = 0;
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
