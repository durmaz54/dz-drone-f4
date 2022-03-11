/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "ibus.h"
#include "motor.h"
#include "mpu9255.h"
#include "cdkit.h"
#include "BMP180.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

int8_t testData;
float temperature;
float pressure;
float altitude;
int16_t frontCm;
uint16_t rcData[16];
char bleData[32];
bool isAutonom = false;
struct motors motors;
MPU9255_t MPU9255;
int16_t yaw, pitch, roll;
int16_t throttle;
float pid_error_temp;
float pid_i_mem_roll, pid_output_roll,
		pid_last_roll_d_error;

int16_t pid_roll_setpoint, pid_pitch_setpoint,pid_yaw_setpoint;

float pid_i_mem_pitch, pid_output_pitch,
		pid_last_pitch_d_error;
float pid_i_mem_yaw, gyro_yaw_input, pid_output_yaw,
		pid_last_yaw_d_error;

int16_t gyro_pitch_input, gyro_roll_input;

float pid_p_gain_roll =	2;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0005;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 40.0;    //18           //Gain setting for the roll D-controller
int pid_max_roll = 500;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = 2;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.0005;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 40;  //Gain setting for the pitch D-controller.
int pid_max_pitch = 500;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 500;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min,
		uint16_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void escCalibration() {
	while (1) {
		ibus_read(&huart2, rcData);
		motors.motor1 = rcData[3];
		motors.motor2 = rcData[3];
		motors.motor3 = rcData[3];
		motors.motor4 = rcData[3];
		motor_Write(&motors);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	}

}

void calculate_pid() {
	//Roll calculations
	pid_error_temp = gyro_roll_input - pid_roll_setpoint;
	pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
	if (pid_i_mem_roll > pid_max_roll)
		pid_i_mem_roll = pid_max_roll;
	else if (pid_i_mem_roll < pid_max_roll * -1)
		pid_i_mem_roll = pid_max_roll * -1;

	pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll
			+ pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
	if (pid_output_roll > pid_max_roll)
		pid_output_roll = pid_max_roll;
	else if (pid_output_roll < pid_max_roll * -1)
		pid_output_roll = pid_max_roll * -1;

	pid_last_roll_d_error = pid_error_temp;

	//Pitch calculations
	pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
	pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
	if (pid_i_mem_pitch > pid_max_pitch)
		pid_i_mem_pitch = pid_max_pitch;
	else if (pid_i_mem_pitch < pid_max_pitch * -1)
		pid_i_mem_pitch = pid_max_pitch * -1;

	pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch
			+ pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
	if (pid_output_pitch > pid_max_pitch)
		pid_output_pitch = pid_max_pitch;
	else if (pid_output_pitch < pid_max_pitch * -1)
		pid_output_pitch = pid_max_pitch * -1;

	pid_last_pitch_d_error = pid_error_temp;

	//Yaw calculations
	pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
	pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
	if (pid_i_mem_yaw > pid_max_yaw)
		pid_i_mem_yaw = pid_max_yaw;
	else if (pid_i_mem_yaw < pid_max_yaw * -1)
		pid_i_mem_yaw = pid_max_yaw * -1;

	pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw
			+ pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
	if (pid_output_yaw > pid_max_yaw)
		pid_output_yaw = pid_max_yaw;
	else if (pid_output_yaw < pid_max_yaw * -1)
		pid_output_yaw = pid_max_yaw * -1;

	pid_last_yaw_d_error = pid_error_temp;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	// yukarıdaki alanda cubemx önce usart2'yi init ediyor sonra dma'yı init ediyordu bu yanlış.
	// doğrusu benim kullandığım olacak.
	motors.timx = &htim4;
	motor_Init(&motors);
	//__HAL_TIM_SET_COUNTER(&htim1, 0);
	//HAL_TIM_Base_Start(&htim1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, ENABLE);

	BMP180_Start();
	/*
	while(1){
		temperature = BMP180_GetTemp();
		altitude = BMP180_GetAlt(0);
		pressure = BMP180_GetPress(0);
		sprintf(bleData, "temp=%.2f m%.2f\n", temperature, altitude);
		HAL_UART_Transmit(&huart6, bleData, sizeof(bleData), 100);
		HAL_Delay(100);
	}
	*/
	while (MPU9255_Init(&hi2c2) == 1) {
		HAL_Delay(100);
	}


	 while(!(rcData[0] == FAILSAFE_OFF)){
	 HAL_Delay(100);
	 ibus_read(&huart2, rcData);
	 }

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET); //led yan

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
		escCalibration();
	}



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

			ibus_read(&huart2, rcData);
			readAll(&hi2c2, &MPU9255);
			dz_hcsr_read(&frontCm);
			altitude = BMP180_GetAlt(0);
			gyro_yaw_input = 0;
			gyro_pitch_input = MPU9255.pitch;
			gyro_roll_input = MPU9255.roll;


			sprintf(bleData, "p= %d r= %d \n", gyro_pitch_input, gyro_roll_input);
			HAL_UART_Transmit(&huart6, bleData, sizeof(bleData), 100);

		throttle = rcData[3];

		if((rcData[5] > 1800) & (rcData[5] < 2100)){
			isAutonom = true;
			cdkit_read(huart6, &pid_yaw_setpoint, &pid_pitch_setpoint, &pid_roll_setpoint);

		}
		else{
			isAutonom = false;
			pid_roll_setpoint = (rcData[1] - 1500) / 30;
			pid_pitch_setpoint = (rcData[2] - 1500) / 30;
			pid_yaw_setpoint = (rcData[4] - 1500) / 30;
		}

		if(frontCm < 60){
			dronePitchBack(&pid_yaw_setpoint, &pid_pitch_setpoint, &pid_roll_setpoint);
			droneStop(&pid_yaw_setpoint, &pid_pitch_setpoint, &pid_roll_setpoint);
			droneSetHeight(70);
			dronePitchForward(&pid_yaw_setpoint, &pid_pitch_setpoint, &pid_roll_setpoint);
			droneSetHeight(200);
			//freertos
		}

		if (rcData[0] == FAILSAFE_ACTIVE) {
			motors.motor1 = 1000;
			motors.motor2 = 1000;
			motors.motor3 = 1000;
			motors.motor4 = 1000;
			motor_Write(&motors);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, ENABLE);
		}
		else if(throttle < 1200){

			motors.motor1 = 1000;
			motors.motor2 = 1000;
			motors.motor3 = 1000;
			motors.motor4 = 1000;
			motor_Write(&motors);

		    pid_i_mem_roll = 0;
		    pid_last_roll_d_error = 0;
		    pid_i_mem_pitch = 0;
		    pid_last_pitch_d_error = 0;
		    pid_i_mem_yaw = 0;
		    pid_last_yaw_d_error = 0;
		}

		else {


			calculate_pid();

			if (throttle > 1800) {
				throttle = 1800;
			}

			motors.motor1 = throttle + pid_output_pitch + pid_output_roll // ön sağ
					+ pid_output_yaw;
			motors.motor2 = throttle - pid_output_pitch - pid_output_roll // arka sol
					+ pid_output_yaw;
			motors.motor3 = throttle + pid_output_pitch - pid_output_roll // ön sol
					- pid_output_yaw;
			motors.motor4 = throttle - pid_output_pitch + pid_output_roll // arka sağ
					- pid_output_yaw;

			/*
			 motors.motor1 = throttle - pid_output_pitch + pid_output_roll // ön sağ
					- pid_output_yaw;
			motors.motor4 = throttle + pid_output_pitch + pid_output_roll // arka sağ
					+ pid_output_yaw;
			motors.motor3 = throttle + pid_output_pitch - pid_output_roll // ön sol
					- pid_output_yaw;
			motors.motor2 = throttle - pid_output_pitch - pid_output_roll // arka sol
					+ pid_output_yaw;
			 */

			if (motors.motor1 < 1100) {
				motors.motor1 = 1100;
			}
			if (motors.motor2 < 1100) {
				motors.motor2 = 1100;
			}
			if (motors.motor3 < 1100) {
				motors.motor3 = 1100;
			}
			if (motors.motor4 < 1100) {
				motors.motor4 = 1100;
			}

			if (motors.motor1 > 2000) {
				motors.motor1 = 2000;
			}
			if (motors.motor2 > 2000) {
				motors.motor2 = 2000;
			}
			if (motors.motor3 > 2000) {
				motors.motor3 = 2000;
			}
			if (motors.motor4 > 2000) {
				motors.motor4 = 2000;
			}

			motor_Write(&motors);


		}
		// else end
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

