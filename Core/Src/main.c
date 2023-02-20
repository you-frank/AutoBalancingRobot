/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Self-balancing Robot
  * I2C1 : MPU6050
  * SPI1 : It was supposed to connect RF-Module(SX1276) but damaged during the test.
  * PA1,2,3 : work with RF-module. RESET, NSS, DI0
  * PB1,2 : L-R Motor PWM
  * PA 8, 9,10,11 : Right Motor Break, Direction, Encoder-A, Encoder-B
  * PB12,13,15,15 : Left  Motor Break, Direction, Encoder-A, Encoder-B
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
MPU6050_t MPU6050;
#define FORWARD true
#define BACKWARD false
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * @brief   mapping value x in range in_min~in_max  to  range of out_min~out_max
 * @retval  double
 */
double map(double x, double in_min, double in_max, double out_min, double out_max)
{
	if(x>in_max)	return out_max;
	else if(x<in_min)	return out_min;

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * @brief   simply confirm val is in range of min ~ max
 * @retval  double
 */
double constrain(double val, double min, double max)
{
	if(val<min)  return min;
	if(val>max)  return max;
	return val;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 65535;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_Pin|NSS_Pin|DIR_R_PA9_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BRK_L_PB12_GPIO_Port, BRK_L_PB12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_L_PB13_GPIO_Port, DIR_L_PB13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BRK_R_PA8_GPIO_Port, BRK_R_PA8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RESET_Pin NSS_Pin BRK_R_PA8_Pin DIR_R_PA9_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|NSS_Pin|BRK_R_PA8_Pin|DIR_R_PA9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO0_Pin ENC_R_B_PA11_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin|ENC_R_B_PA11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BRK_L_PB12_Pin DIR_L_PB13_Pin */
  GPIO_InitStruct.Pin = BRK_L_PB12_Pin|DIR_L_PB13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_L_A_PB14_ITR_Pin */
  GPIO_InitStruct.Pin = ENC_L_A_PB14_ITR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_L_A_PB14_ITR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_L_B_PB15_Pin */
  GPIO_InitStruct.Pin = ENC_L_B_PB15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_L_B_PB15_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_R_A_PA10_ITR_Pin */
  GPIO_InitStruct.Pin = ENC_R_A_PA10_ITR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_R_A_PA10_ITR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//
/**
  * @brief  Tow Encoder EnA pin interrupt callback. Prepared for read motor encoder, but not using currently.
  * @param  Encoder EnA pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  *
  * Positive MoterPower : heading to jtag connector direction
  * Negative MoterPower : heading to usb connector direction
  * Lean to JTAG direction : Increasing current Angle
  * Lean to USB direction : Decreasing current Angle.
  *
  */
#define Kp 62
#define Ki 600
#define Kd 0.3

#define TARGET_ANGLE -2.5 //-1.5

volatile double accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0, self_balance_pid_setpoint=0;
volatile int gyroRate;

float targetAngle = TARGET_ANGLE;
uint32_t gyro_tick, motor_tick;
bool b_fall=false;

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	uint32_t cur_tick;
	uint8_t direction=false;   // false :to Ring, true:to sensors
	int16_t accY, accZ, gyroX;
	int16_t motorPower;
	float sampleTime;

	HAL_TIM_Base_Start(&htim3);               //Initialize stm32 timer 3
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);  //PB0 Start pwm second motor 100% duty cycle
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);  //PB1 Start pwm first motor  100% duty cycle

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, direction); // Green Direction
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, !direction);  // Red Direction

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // Release Break
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // Release Break    BRK_L_PB12_GPIO_Port

	while (MPU6050_Init(&hi2c1) == 1);

	//MPU6050_Calib_GX_AYZ(&hi2c1, &MPU6050); // Calibrate MPU6050 for get offsets, target angle.
	MPU6050.GyroX_offset=-136; // = (average Gyro_X_RAW)/4
	MPU6050.AccY_offset = 527;//522;//7;(jtag방향 약간 기움)   커지면 usb, 작아지면 jtag
	MPU6050.AccZ_offset = -318;//-162;// // = ( 16384- average ACC_Z_RAW)/8
	osDelay(500);

	for(;;)
	{
		cur_tick=HAL_GetTick();

		if(cur_tick-gyro_tick>5){
			if(gyro_tick==0)
				gyro_tick=cur_tick-5;

			sampleTime=(float)(cur_tick-gyro_tick)/1000;
			//MPU6050_Read_All(&hi2c1, &MPU6050);
			MPU6050_Read_GyroX_AccYZ_RAW(&hi2c1, &MPU6050); // reading just what I need. (accY, accZ, gyroX)

			if(MPU6050.Accel_Y_RAW==0 && MPU6050.Accel_Z_RAW==0 && MPU6050.Gyro_X_RAW==0){ // Should check. fail to read sometime.
				// Wait 1msec then start over.
				osDelay(1);
				continue;
			}
			accY = MPU6050.Accel_Y_RAW - MPU6050.AccY_offset;
			accZ = MPU6050.Accel_Z_RAW - MPU6050.AccZ_offset;
			gyroX = MPU6050.Gyro_X_RAW - MPU6050.GyroX_offset;
			accAngle = atan2(accY, accZ)*RAD_TO_DEG;
			gyroRate = map(gyroX, -32768, 32767, -250, 250);
			gyroAngle = (float)gyroRate*sampleTime;

			// Complementary Filter with sensor reading angle.
			float comp_h = 0.75/(0.75+sampleTime);
			//currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
			currentAngle = comp_h*(prevAngle + gyroAngle) + (1-comp_h)*(accAngle);

			error = currentAngle - targetAngle-self_balance_pid_setpoint;
			errorSum = errorSum + error;
			errorSum = constrain(errorSum, -600, 600);

			// compute PID for updating motorPower.
			motorPower = (int16_t)(Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime);

			prevAngle = currentAngle;

			// motor stop when robot fall.
			if(accY>9000 || accY<-9000){  // accY(-9000)=kalmanX(-29)=currentAngle(-50),      accY(9000)=kalX(29)=currentAngle(5.5)       KalX(0)=currentAngle(-23)
				motorPower=0;
				prevAngle=0;
				errorSum=0;
				self_balance_pid_setpoint=0;
				b_fall=true;
			}else
				b_fall=false;

			motorPower = constrain(motorPower, -1000, 1000);
			// set motor direction
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, motorPower>0?FORWARD:BACKWARD); // green
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  motorPower>0?BACKWARD:FORWARD); // red
			if(b_fall)
				motorPower=0;
			// set motor speed.
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 1000-abs(motorPower)); // green
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 1000-abs(motorPower)); // red

			gyro_tick=cur_tick;
		}
		if(cur_tick-motor_tick>10){
			//setMotorSpeed(cur_speed,0);
			motor_tick=cur_tick;
		}
	}
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
  while (1)
  {
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

