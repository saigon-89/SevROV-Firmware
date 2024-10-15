/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include "lsm6ds33.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
  uint8_t MagicStart;
  uint64_t Flags;
  
  float DesMotServo[6];
  float DesCamServo;
	float DesLED[2];
  uint8_t Padding[154];
  
  uint8_t MagicEnd;
} SPI_ControlPackageTypeDef;

typedef struct __attribute__((packed)) {
  uint8_t MagicStart;
  uint64_t Flags;
  
  float IMUEuler[3];
	float IMUAccel[3];
	float IMUMagnet[3];
  float CurrentGeneral;
	float CurrentLightLeft;
	float CurrentLightRight;
	float Voltage24V;
  uint8_t Padding[138];
  
  uint8_t MagicEnd;
} SPI_TelemetryPackageTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_BUFFER_SIZE sizeof(SPI_ControlPackageTypeDef)

#define SPI_CONTROL_PACKAGE_DES_MOT_SERVOx_FLAG (1 << 0)
#define SPI_CONTROL_PACKAGE_DES_CAM_SERVO_FLAG (1 << 1)
#define SPI_CONTROL_PACKAGE_DES_LED_FLAG (1 << 2)

#define SPI_TELEMETRY_PACKAGE_IMU_EULERx_FLAG (1 << 0)
#define SPI_TELEMETRY_PACKAGE_IMU_ACCELx_FLAG (1 << 1)
#define SPI_TELEMETRY_PACKAGE_IMU_MAGNETx_FLAG (1 << 2)
#define SPI_TELEMETRY_PACKAGE_CURRENT_GENERAL_FLAG (1 << 3)
#define SPI_TELEMETRY_PACKAGE_CURRENT_LIGHT_LEFT_FLAG (1 << 4)
#define SPI_TELEMETRY_PACKAGE_CURRENT_LIGHT_RIGHT_FLAG (1 << 5)
#define SPI_TELEMETRY_PACKAGE_VOLTAGE_24V_FLAG (1 << 6)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
LSM6_HandleTypeDef hlsm6;

static SPI_ControlPackageTypeDef spi1_rx_buf = { 0 };
static SPI_TelemetryPackageTypeDef spi1_tx_buf = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static float Clamp(float value, float min, float max);

static uint16_t DutyCycleToCCRx(float duty);

static uint16_t ServoLoadToCCRx(float load);

void PWM_MOT1_SetServo(float load);
void PWM_MOT2_SetServo(float load);
void PWM_MOT3_SetServo(float load);
void PWM_MOT4_SetServo(float load);
void PWM_MOT5_SetServo(float load);
void PWM_MOT6_SetServo(float load);
void PWM_MOT7_SetServo(float load);
void PWM_MOT8_SetServo(float load);
HAL_StatusTypeDef PWM_MOTx_SetServo(uint8_t idx, float load);

void PWM_MAN1_SetDutyCycle(float duty);
void PWM_MAN2_SetDutyCycle(float duty);

void PWM_LED_SetDutyCycle(float duty);

void PWM_SERVO_SetServo(float load);

HAL_StatusTypeDef PACKAGE_GetDesMotServo(SPI_ControlPackageTypeDef *package, float servo[6]);
HAL_StatusTypeDef PACKAGE_GetDesCamServo(SPI_ControlPackageTypeDef *package, float *servo);
HAL_StatusTypeDef PACKAGE_GetDesLED(SPI_ControlPackageTypeDef *package, float led[2]);

HAL_StatusTypeDef PACKAGE_SetIMUEuler(SPI_TelemetryPackageTypeDef *package, float euler[3]);
HAL_StatusTypeDef PACKAGE_SetIMUAccel(SPI_TelemetryPackageTypeDef *package, float accel[3]);
HAL_StatusTypeDef PACKAGE_SetIMUMagnet(SPI_TelemetryPackageTypeDef *package, float magnet[3]);
HAL_StatusTypeDef PACKAGE_SetCurrentGeneral(SPI_TelemetryPackageTypeDef *package, float current);
HAL_StatusTypeDef PACKAGE_SetCurrentLightLeft(SPI_TelemetryPackageTypeDef *package, float current);
HAL_StatusTypeDef PACKAGE_SetCurrentLightRight(SPI_TelemetryPackageTypeDef *package, float current);
HAL_StatusTypeDef PACKAGE_SetVoltage24V(SPI_TelemetryPackageTypeDef *package, float voltage);

HAL_StatusTypeDef PACKAGE_TelemetryPackageInit(SPI_TelemetryPackageTypeDef *package);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float Clamp(float value, float min, float max) {
  return (value < min) ? min : (value > max) ? max : value;
}

static uint16_t DutyCycleToCCRx(float duty) {
	duty = Clamp(duty, 0.0, 100.0);
  return (uint16_t)((float)UINT16_MAX * (duty / 100.0f));
}

static uint16_t ServoLoadToCCRx(float load) {
	load = Clamp(load, -100.0, 100.0);
	float in_min = -100.0;
	float in_max = 100.0;
	float out_min = 5.0;
	float out_max = 10.0;
	float duty = (load - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return DutyCycleToCCRx(duty);
}

void PWM_MOT1_SetServo(float load) {
  WRITE_REG(TIM1->CCR3, ServoLoadToCCRx(load));
}

void PWM_MOT2_SetServo(float load) {
  WRITE_REG(TIM1->CCR2, ServoLoadToCCRx(load));
}

void PWM_MOT3_SetServo(float load) {
  WRITE_REG(TIM1->CCR1, ServoLoadToCCRx(load));
}

void PWM_MAN1_SetDutyCycle(float duty) {
	WRITE_REG(TIM2->CCR2, DutyCycleToCCRx(duty));
}

void PWM_MAN2_SetDutyCycle(float duty) {
	WRITE_REG(TIM2->CCR3, DutyCycleToCCRx(duty));
}

void PWM_MOT8_SetServo(float load) {
  WRITE_REG(TIM2->CCR4, ServoLoadToCCRx(load));
}

void PWM_MOT4_SetServo(float load) {
  WRITE_REG(TIM3->CCR4, ServoLoadToCCRx(load));
}

void PWM_MOT5_SetServo(float load) {
  WRITE_REG(TIM3->CCR3, ServoLoadToCCRx(load));
}

void PWM_MOT6_SetServo(float load) {
  WRITE_REG(TIM3->CCR2, ServoLoadToCCRx(load));
}

void PWM_MOT7_SetServo(float load) {
  WRITE_REG(TIM3->CCR1, ServoLoadToCCRx(load));
}

void PWM_LED_SetDutyCycle(float duty) {
	WRITE_REG(TIM4->CCR4, DutyCycleToCCRx(duty));
}

void PWM_SERVO_SetServo(float load) {
	WRITE_REG(TIM4->CCR3, ServoLoadToCCRx(load));
}

HAL_StatusTypeDef PWM_MOTx_SetServo(uint8_t idx, float load) {
  if (idx >= 6) return HAL_ERROR;
	if (idx == 0) {
	  PWM_MOT3_SetServo(load);
	} else if (idx == 1) {
		PWM_MOT4_SetServo(load);
	} else if (idx == 2) {
		PWM_MOT5_SetServo(load);
	} else if (idx == 3) {
		PWM_MOT6_SetServo(load);
	} else if (idx == 4) {
		PWM_MOT7_SetServo(load);
	} else if (idx == 5) {
		PWM_MOT8_SetServo(load);
	}
	return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_GetDesMotServo(SPI_ControlPackageTypeDef *package, float servo[6]) {
  if (package->Flags & SPI_CONTROL_PACKAGE_DES_MOT_SERVOx_FLAG) {
		for (size_t idx = 0; idx < 6; idx++) {
			servo[idx] = package->DesMotServo[idx];
		}
    package->Flags &= ~SPI_CONTROL_PACKAGE_DES_MOT_SERVOx_FLAG;
    return HAL_OK;
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef PACKAGE_GetDesCamServo(SPI_ControlPackageTypeDef *package, float *servo) {
  if (package->Flags & SPI_CONTROL_PACKAGE_DES_CAM_SERVO_FLAG) {
    *servo = package->DesCamServo;
    package->Flags &= ~SPI_CONTROL_PACKAGE_DES_CAM_SERVO_FLAG;
    return HAL_OK;
  }
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_GetDesLED(SPI_ControlPackageTypeDef *package, float led[2]) {
  if (package->Flags & SPI_CONTROL_PACKAGE_DES_LED_FLAG) {
		for (size_t idx = 0; idx < 2; idx++) {
			led[idx] = package->DesLED[idx];
		}
    package->Flags &= ~SPI_CONTROL_PACKAGE_DES_LED_FLAG;
    return HAL_OK;
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef PACKAGE_SetIMUEuler(SPI_TelemetryPackageTypeDef *package, float euler[3]) {
	for (size_t idx = 0; idx < 3; idx++) {
		package->IMUEuler[idx] = euler[idx];
	}
  package->Flags |= SPI_TELEMETRY_PACKAGE_IMU_EULERx_FLAG;
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetIMUAccel(SPI_TelemetryPackageTypeDef *package, float accel[3]) {
	for (size_t idx = 0; idx < 3; idx++) {
		package->IMUAccel[idx] = accel[idx];
	}
  package->Flags |= SPI_TELEMETRY_PACKAGE_IMU_ACCELx_FLAG;
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetIMUMagnet(SPI_TelemetryPackageTypeDef *package, float magnet[3]) {
	for (size_t idx = 0; idx < 3; idx++) {
		package->IMUMagnet[idx] = magnet[idx];
	}
  package->Flags |= SPI_TELEMETRY_PACKAGE_IMU_MAGNETx_FLAG;
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetCurrentGeneral(SPI_TelemetryPackageTypeDef *package, float current) {
	package->CurrentGeneral = current;
  package->Flags |= SPI_TELEMETRY_PACKAGE_CURRENT_GENERAL_FLAG;
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetCurrentLightLeft(SPI_TelemetryPackageTypeDef *package, float current) {
	package->CurrentLightLeft = current;
  package->Flags |= SPI_TELEMETRY_PACKAGE_CURRENT_LIGHT_LEFT_FLAG;
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetCurrentLightRight(SPI_TelemetryPackageTypeDef *package, float current) {
	package->CurrentLightRight = current;
  package->Flags |= SPI_TELEMETRY_PACKAGE_CURRENT_LIGHT_RIGHT_FLAG;
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetVoltage24V(SPI_TelemetryPackageTypeDef *package, float voltage) {
	package->Voltage24V = voltage;
  package->Flags |= SPI_TELEMETRY_PACKAGE_VOLTAGE_24V_FLAG;
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_TelemetryPackageInit(SPI_TelemetryPackageTypeDef *package) {
  if (package == NULL) return HAL_ERROR;
	package->MagicStart = 0xAB;
	package->MagicEnd = 0xCD;
	return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_ControlPackageCheck(SPI_ControlPackageTypeDef *package) {
  if (package == NULL) return HAL_ERROR;
	if (package->MagicStart != 0xAB) return HAL_ERROR;
	if (package->MagicEnd != 0xCD) return HAL_ERROR;
	return HAL_OK;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PACKAGE_TelemetryPackageInit(&spi1_tx_buf);
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	LSM6_Init(&hlsm6, &hi2c2);
	LSM6_Enable_Default(&hlsm6);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		float euler[] = { 13.5, 10.1, 53.1 };
		float accel[] = { 0.23, 0.12, 9.53 };
		float magnet[] = { 1.27, 2.56, 6.12 };
		PACKAGE_SetIMUEuler(&spi1_tx_buf, euler);
		PACKAGE_SetIMUAccel(&spi1_tx_buf, accel);
		PACKAGE_SetIMUMagnet(&spi1_tx_buf, magnet);
		PACKAGE_SetCurrentGeneral(&spi1_tx_buf, 0.250);
		PACKAGE_SetCurrentLightLeft(&spi1_tx_buf, 0.242);
		PACKAGE_SetCurrentLightRight(&spi1_tx_buf, 0.234);
		PACKAGE_SetVoltage24V(&spi1_tx_buf, 23.85);
		
		printf("+++++++++++++++++++++++\r\n");
#if 0
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&spi1_tx_buf, (uint8_t *)&spi1_rx_buf, SPI_BUFFER_SIZE, HAL_MAX_DELAY);
		spi1_tx_buf.Flags = 0;
		if (PACKAGE_ControlPackageCheck(&spi1_rx_buf) == HAL_OK) {
		  float mot_servo[6];
			float cam_servo;
			float led[2];
			if (PACKAGE_GetDesMotServo(&spi1_rx_buf, mot_servo) == HAL_OK) {
				for (uint8_t idx = 0; idx < 6; idx++) {
					printf("MOT_SERVO[%d]: %.2f\r\n", idx, mot_servo[idx]);
					PWM_MOTx_SetServo(idx, mot_servo[idx]);
				}
			}

			if (PACKAGE_GetDesCamServo(&spi1_rx_buf, &cam_servo) == HAL_OK) {
				printf("CAM_SERVO: %.2f\r\n", cam_servo);
			}
			
			if (PACKAGE_GetDesLED(&spi1_rx_buf, led) == HAL_OK) {
				for (uint8_t idx = 0; idx < 2; idx++) {
					printf("LED[%d]: %.2f\r\n", idx, led[idx]);
				}
			}
		}
#endif
		LSM6_Read(&hlsm6);
		printf("ACCEL[x]: %d\n", hlsm6.Accel.x);
		printf("ACCEL[y]: %d\n", hlsm6.Accel.y);
		printf("ACCEL[z]: %d\n", hlsm6.Accel.z);
		HAL_Delay(100);
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c2.Init.ClockSpeed = 100000;
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 21;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 21;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 21;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 21;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Dir1_Pin|Dir2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(En__GPIO_Port, En__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Dir1_Pin Dir2_Pin */
  GPIO_InitStruct.Pin = Dir1_Pin|Dir2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : En__Pin */
  GPIO_InitStruct.Pin = En__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(En__GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
