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
#include <string.h>
#include <stdbool.h>

#include "lsm6ds33.h"
#include "lis3mdl.h"
#include "minimu9.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FOC_DRIVERS_MODE
#define HYBRID_ROV_MODE

#define MAX_FLOAT16 (128.0f)
#define MIN_FLOAT16 (-MAX_FLOAT16)

#define UWMANIPULATOR_RPDO_COB_ID (0x200)
#define UWMANIPULATOR_CMD_ESC_FREQ (0x20)
#define UWMANIPULATOR_CMD_ESC_RPM (0x24)
#define UWMANIPULATOR_CMD_SERVO (0x30)
#define UWMANIPULATOR_CMD_SERVO_STEPPER (0x38)
#define UWMANIPULATOR_CMD_CURRENT (0x10)
#define UWMANIPULATOR_CMD_TORGUE (0x14)

#define UWMANIPULATOR_UNITS 3

typedef enum {
  UWMANIPULATOR_GRIP_OPEN = 0,
	UWMANIPULATOR_GRIP_CLOSE = 1,
	UWMANIPULATOR_GRIP_STOP = 2,
} UWManipulator_GripStateTypeDef;

typedef struct {
  struct {
    uint16_t fault_bits;
    float voltage_dc;
    float electrical_freq;
  } id18x_data[UWMANIPULATOR_UNITS];
  struct {
    float electrical_position;
    float workzone_counts;
  } id28x_data[UWMANIPULATOR_UNITS];
  struct {
    float phase_a_current;
    float phase_b_current;
  } id38x_data[UWMANIPULATOR_UNITS];
	float q[UWMANIPULATOR_UNITS];
  UWManipulator_GripStateTypeDef grip_state;
} UWManipulator_HandleTypeDef;

typedef struct __attribute__((packed)) {
  uint8_t MagicStart;
  uint64_t Flags;

  float DesMotServo[6];
  float DesCamServo;
	float DesLED[2];

#ifdef HYBRID_ROV_MODE
	float DesManQ[3];
	uint8_t DesManGripState;
	uint8_t Padding[141];
#else
	uint8_t Padding[154];
#endif

  uint8_t MagicEnd;
} SPI_ControlPackageTypeDef;

typedef struct __attribute__((packed)) {
  uint8_t MagicStart;
  uint64_t Flags;

  float IMUEuler[3];
	float IMUAccel[3];
	float IMUGyro[3];
	float IMUMagnet[3];
  float CurrentGeneral;
	float CurrentLightLeft;
	float CurrentLightRight;
	float Voltage24V;

	struct __attribute__((packed)) {
    float Ia;
	  float Ib;
	  float Ic;
  } MotTelemetry[6];

#ifdef HYBRID_ROV_MODE
	struct __attribute__((packed)) {
    float Ia;
	  float Ib;
	  float Voltage;
		float q;
  } ManTelemetry[3];
	uint8_t Padding[6];
#else
	uint8_t Padding[54];
#endif

  uint8_t MagicEnd;
} SPI_TelemetryPackageTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNELS_NUM 6

#define SPI_BUFFER_SIZE sizeof(SPI_ControlPackageTypeDef)

#define SPI_CONTROL_PACKAGE_DES_MOT_SERVOx_FLAG (1 << 0)
#define SPI_CONTROL_PACKAGE_DES_CAM_SERVO_FLAG (1 << 1)
#define SPI_CONTROL_PACKAGE_DES_LED_FLAG (1 << 2)

#define SPI_CONTROL_PACKAGE_DES_MAN_Qx_FLAG(x) (1 << (3 + x))
#define SPI_CONTROL_PACKAGE_DES_MAN_GRIP_FLAG (1 << 6)

#define SPI_TELEMETRY_PACKAGE_IMU_EULERx_FLAG (1 << 0)
#define SPI_TELEMETRY_PACKAGE_IMU_ACCELx_FLAG (1 << 1)
#define SPI_TELEMETRY_PACKAGE_IMU_GYROx_FLAG (1 << 2)
#define SPI_TELEMETRY_PACKAGE_IMU_MAGNETx_FLAG (1 << 3)
#define SPI_TELEMETRY_PACKAGE_CURRENT_GENERAL_FLAG (1 << 4)
#define SPI_TELEMETRY_PACKAGE_CURRENT_LIGHT_LEFT_FLAG (1 << 5)
#define SPI_TELEMETRY_PACKAGE_CURRENT_LIGHT_RIGHT_FLAG (1 << 6)
#define SPI_TELEMETRY_PACKAGE_VOLTAGE_24V_FLAG (1 << 7)

#define SPI_TELEMETRY_PACKAGE_MOTx_PHASE_A_FLAG(x) (1 << (8 + x))
#define SPI_TELEMETRY_PACKAGE_MOTx_PHASE_B_FLAG(x) (1 << (14 + x))
#define SPI_TELEMETRY_PACKAGE_MOTx_PHASE_C_FLAG(x) (1 << (20 + x))

#ifdef HYBRID_ROV_MODE
#define SPI_TELEMETRY_PACKAGE_MAN_UNITx_VOLTAGE_FLAG(x) (1 << (26 + x))
#define SPI_TELEMETRY_PACKAGE_MAN_UNITx_PHASES_A_B_FLAG(x) (1 << (29 + x))
#define SPI_TELEMETRY_PACKAGE_MAN_UNITx_ANGLE_FLAG(x) (1 << (32 + x))
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
LSM6_HandleTypeDef hlsm6;
LIS3MDL_HandleTypeDef hlis3mdl;
MINIMU9_HandleTypeDef hminimu9;
UWManipulator_HandleTypeDef huwman;

static SPI_ControlPackageTypeDef spi_rx_buf = { 0 };
static SPI_TelemetryPackageTypeDef spi_tx_buf = { 0 };
static bool spiTxRxCpltStatus = true;

static uint16_t adcData[ADC_CHANNELS_NUM];
static bool adcConvCpltStatus = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
int stdout_putchar(int ch);

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

HAL_StatusTypeDef PACKAGE_GetDesMotCtrl(SPI_ControlPackageTypeDef *package, float servo[6]);
HAL_StatusTypeDef PACKAGE_GetDesCamServo(SPI_ControlPackageTypeDef *package, float *servo);
HAL_StatusTypeDef PACKAGE_GetDesLED(SPI_ControlPackageTypeDef *package, float led[2]);

#ifdef HYBRID_ROV_MODE
HAL_StatusTypeDef PACKAGE_GetDesManQ(SPI_ControlPackageTypeDef *package, uint8_t unit, float *q);
HAL_StatusTypeDef PACKAGE_GetDesManGripState(SPI_ControlPackageTypeDef *package, uint8_t *state);
#endif

HAL_StatusTypeDef PACKAGE_SetIMUEuler(SPI_TelemetryPackageTypeDef *package, float euler[3]);
HAL_StatusTypeDef PACKAGE_SetIMUAccel(SPI_TelemetryPackageTypeDef *package, float accel[3]);
HAL_StatusTypeDef PACKAGE_SetIMUGyro(SPI_TelemetryPackageTypeDef *package, float gyro[3]);
HAL_StatusTypeDef PACKAGE_SetIMUMagnet(SPI_TelemetryPackageTypeDef *package, float magnet[3]);
HAL_StatusTypeDef PACKAGE_SetCurrentGeneral(SPI_TelemetryPackageTypeDef *package, float current);
HAL_StatusTypeDef PACKAGE_SetCurrentLightLeft(SPI_TelemetryPackageTypeDef *package, float current);
HAL_StatusTypeDef PACKAGE_SetCurrentLightRight(SPI_TelemetryPackageTypeDef *package, float current);
HAL_StatusTypeDef PACKAGE_SetVoltage24V(SPI_TelemetryPackageTypeDef *package, float voltage);

#ifdef HYBRID_ROV_MODE
HAL_StatusTypeDef PACKAGE_SetMotCurrentPhaseA(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float current);
HAL_StatusTypeDef PACKAGE_SetMotCurrentPhaseB(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float current);
HAL_StatusTypeDef PACKAGE_SetMotCurrentPhaseC(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float current);
HAL_StatusTypeDef PACKAGE_SetManVoltage(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float voltage);
HAL_StatusTypeDef PACKAGE_SetManCurrents(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float current[2]);
HAL_StatusTypeDef PACKAGE_SetManQ(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float q);
#endif

HAL_StatusTypeDef PACKAGE_TelemetryPackageInit(SPI_TelemetryPackageTypeDef *package);

#ifdef HYBRID_ROV_MODE
static HAL_StatusTypeDef UWManipulator_SetRPM(UWManipulator_HandleTypeDef *huwman, uint8_t unit, float rpm);
static HAL_StatusTypeDef UWManipulator_SetStep(UWManipulator_HandleTypeDef *huwman, uint8_t unit, float step);
#endif
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

HAL_StatusTypeDef PACKAGE_GetDesMotCtrl(SPI_ControlPackageTypeDef *package, float servo[6]) {
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

#ifdef HYBRID_ROV_MODE
HAL_StatusTypeDef PACKAGE_GetDesManQ(SPI_ControlPackageTypeDef *package, uint8_t unit, float *q) {
  if (package->Flags & SPI_CONTROL_PACKAGE_DES_MAN_Qx_FLAG(unit)) {
		*q = package->DesManQ[unit];
    package->Flags &= ~SPI_CONTROL_PACKAGE_DES_MAN_Qx_FLAG(unit);
    return HAL_OK;
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef PACKAGE_GetDesManGripState(SPI_ControlPackageTypeDef *package, uint8_t *state) {
  if (package->Flags & SPI_CONTROL_PACKAGE_DES_MAN_GRIP_FLAG) {
		*state = package->DesManGripState;
    package->Flags &= ~SPI_CONTROL_PACKAGE_DES_MAN_GRIP_FLAG;
    return HAL_OK;
  }
  return HAL_ERROR;
}
#endif

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

HAL_StatusTypeDef PACKAGE_SetIMUGyro(SPI_TelemetryPackageTypeDef *package, float gyro[3]) {
	for (size_t idx = 0; idx < 3; idx++) {
		package->IMUGyro[idx] = gyro[idx];
	}
  package->Flags |= SPI_TELEMETRY_PACKAGE_IMU_GYROx_FLAG;
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

HAL_StatusTypeDef PACKAGE_SetMotCurrentPhaseA(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float current) {
	package->MotTelemetry[unit].Ia = current;
  package->Flags |= SPI_TELEMETRY_PACKAGE_MOTx_PHASE_A_FLAG(unit);
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetMotCurrentPhaseB(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float current) {
	package->MotTelemetry[unit].Ib = current;
  package->Flags |= SPI_TELEMETRY_PACKAGE_MOTx_PHASE_B_FLAG(unit);
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetMotCurrentPhaseC(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float current) {
	package->MotTelemetry[unit].Ic = current;
  package->Flags |= SPI_TELEMETRY_PACKAGE_MOTx_PHASE_C_FLAG(unit);
  return HAL_OK;
}

#ifdef HYBRID_ROV_MODE
HAL_StatusTypeDef PACKAGE_SetManVoltage(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float voltage) {
	package->ManTelemetry[unit].Voltage = voltage;
  package->Flags |= SPI_TELEMETRY_PACKAGE_MAN_UNITx_VOLTAGE_FLAG(unit);
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetManCurrents(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float current[2]) {
	package->ManTelemetry[unit].Ia = current[0];
	package->ManTelemetry[unit].Ib = current[1];
  package->Flags |= SPI_TELEMETRY_PACKAGE_MAN_UNITx_PHASES_A_B_FLAG(unit);
  return HAL_OK;
}

HAL_StatusTypeDef PACKAGE_SetManAngle(SPI_TelemetryPackageTypeDef *package, uint8_t unit, float angle) {
	package->ManTelemetry[unit].q = angle;
  package->Flags |= SPI_TELEMETRY_PACKAGE_MAN_UNITx_ANGLE_FLAG(unit);
  return HAL_OK;
}
#endif

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

static HAL_StatusTypeDef FOC_MOTx_SetRPM(uint8_t idx, float rpm) {
	if (idx >= 6) return HAL_ERROR;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	CAN_TxHeaderTypeDef msgHeader;
	msgHeader.StdId = 0x521 + idx;
	msgHeader.DLC = 8;
	msgHeader.TransmitGlobalTime = DISABLE;
	msgHeader.RTR = CAN_RTR_DATA;
	msgHeader.IDE = CAN_ID_STD;
		
	uint32_t mailBoxNum = 0;
	uint8_t msgData[8] = {0};
	memcpy(msgData, &rpm, sizeof(float));
		
	HAL_CAN_AddTxMessage(&hcan, &msgHeader, msgData, &mailBoxNum);

	return HAL_OK;
}

#ifdef HYBRID_ROV_MODE
HAL_StatusTypeDef UWManipulator_UpdateTelemetryID18x(UWManipulator_HandleTypeDef *huwman,
                                                     uint8_t *data, uint8_t unit) {
  if (data != NULL) {
		memcpy(&huwman->id18x_data[unit].fault_bits, &data[0], sizeof(uint16_t));
    int16_t voltage_dc_bits = ((int16_t)data[3] << 8) | data[2];
    huwman->id18x_data[unit].voltage_dc = (float)(voltage_dc_bits) * (MAX_FLOAT16 / 32767.0f);
    memcpy(&huwman->id18x_data[unit].electrical_freq, &data[4], sizeof(float));
  }

  return HAL_OK;
}

HAL_StatusTypeDef UWManipulator_UpdateTelemetryID28x(UWManipulator_HandleTypeDef *huwman,
                                                     uint8_t *data, uint8_t unit) {
  if (data != NULL) {
		memcpy(&huwman->id28x_data[unit].electrical_position, &data[0], sizeof(float));
		memcpy(&huwman->id28x_data[unit].workzone_counts, &data[4], sizeof(float));
		huwman->q[unit] = 2.0f * 3.14f * huwman->id28x_data[unit].workzone_counts / 131072.0f;
  }

  return HAL_OK;
}

HAL_StatusTypeDef UWManipulator_UpdateTelemetryID38x(UWManipulator_HandleTypeDef *huwman,
                                                     uint8_t *data, uint8_t unit) {
  if (data != NULL) {
		memcpy(&huwman->id38x_data[unit].phase_a_current, &data[0], sizeof(float));
		memcpy(&huwman->id38x_data[unit].phase_b_current, &data[4], sizeof(float));
  }

  return HAL_OK;
}
																										 
static HAL_StatusTypeDef UWManipulator_SetRPM(UWManipulator_HandleTypeDef *huwman, uint8_t unit, float rpm) {
	if (unit >= 3) return HAL_ERROR;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	uint8_t msgData[8] = {0};
	uint32_t mailBoxNum = 0;
	CAN_TxHeaderTypeDef msgHeader;
	msgHeader.StdId = UWMANIPULATOR_RPDO_COB_ID | unit | 0x001;
	msgHeader.DLC = 8;
	msgHeader.TransmitGlobalTime = DISABLE;
	msgHeader.RTR = CAN_RTR_DATA;
	msgHeader.IDE = CAN_ID_STD;

  msgData[0] = UWMANIPULATOR_CMD_ESC_RPM;
  memcpy(&msgData[4], &rpm, sizeof(float));

  HAL_CAN_AddTxMessage(&hcan, &msgHeader, msgData, &mailBoxNum);

	return HAL_OK;
}

static HAL_StatusTypeDef UWManipulator_SetPosition(UWManipulator_HandleTypeDef *huwman, uint8_t unit, float pose) {
	if (unit >= 3) return HAL_ERROR;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	uint8_t msgData[8] = {0};
	uint32_t mailBoxNum = 0;
	CAN_TxHeaderTypeDef msgHeader;
	msgHeader.StdId = UWMANIPULATOR_RPDO_COB_ID | unit | 0x001;
	msgHeader.DLC = 8;
	msgHeader.TransmitGlobalTime = DISABLE;
	msgHeader.RTR = CAN_RTR_DATA;
	msgHeader.IDE = CAN_ID_STD;

	float counts = pose * 131072.0f / (2.0f * 3.14f);
  msgData[0] = UWMANIPULATOR_CMD_SERVO;
	memcpy(&msgData[4], &counts, sizeof(float));

	HAL_CAN_AddTxMessage(&hcan, &msgHeader, msgData, &mailBoxNum);

	return HAL_OK;
}

static HAL_StatusTypeDef UWManipulator_SetStep(UWManipulator_HandleTypeDef *huwman, uint8_t unit, float step) {
	if (unit >= 3) return HAL_ERROR;

	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	uint8_t msgData[8] = {0};
	uint32_t mailBoxNum = 0;
	CAN_TxHeaderTypeDef msgHeader;
	msgHeader.StdId = UWMANIPULATOR_RPDO_COB_ID | unit | 0x001;
	msgHeader.DLC = 8;
	msgHeader.TransmitGlobalTime = DISABLE;
	msgHeader.RTR = CAN_RTR_DATA;
	msgHeader.IDE = CAN_ID_STD;

  msgData[0] = UWMANIPULATOR_CMD_SERVO_STEPPER;
  memcpy(&msgData[4], &step, sizeof(float));

	HAL_CAN_AddTxMessage(&hcan, &msgHeader, msgData, &mailBoxNum);

	return HAL_OK;
}

static HAL_StatusTypeDef UWManipulator_SetGripState(UWManipulator_HandleTypeDef *huwman, uint8_t state) {
	if (state == UWMANIPULATOR_GRIP_OPEN) {
		PWM_MOT1_SetServo(85.0f);
	} else if (state == UWMANIPULATOR_GRIP_CLOSE) {
		PWM_MOT1_SetServo(-85.0f);
	} else if (state == UWMANIPULATOR_GRIP_STOP) {
		PWM_MOT1_SetServo(0.0f);
	} else {
	  return HAL_ERROR;
	}

  return HAL_OK;
}
#endif

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    spi_tx_buf.Flags = 0;
    spiTxRxCpltStatus = true;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef msgHeader;
  uint8_t msgData[8];
	
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msgHeader, msgData);

#ifdef HYBRID_ROV_MODE
	for (size_t unit = 0; unit < UWMANIPULATOR_UNITS; unit++) {
	  if (msgHeader.StdId == (0x181 + unit)) {
	    UWManipulator_UpdateTelemetryID18x(&huwman, msgData, unit);
			float voltage = huwman.id18x_data[unit].voltage_dc;
			PACKAGE_SetManVoltage(&spi_tx_buf, unit, voltage);
		  return;
	  }

	  if (msgHeader.StdId == (0x281 + unit)) {
	    UWManipulator_UpdateTelemetryID28x(&huwman, msgData, unit);
			PACKAGE_SetManAngle(&spi_tx_buf, unit, huwman.q[unit]);
	  	return;
	  }

	  if (msgHeader.StdId == (0x381 + unit)) {
	    UWManipulator_UpdateTelemetryID38x(&huwman, msgData, unit);
			float currents[] = {
				huwman.id38x_data[unit].phase_a_current,
				huwman.id38x_data[unit].phase_b_current
			};
			PACKAGE_SetManCurrents(&spi_tx_buf, unit, currents);
		  return;
	  }
	}
#endif

	for (size_t idx = 0; idx < 6; idx++) {
		if (msgHeader.StdId == (0x481 + idx)) {
			float phase_current = 0;
			memcpy(&phase_current, msgData, sizeof(float));
			PACKAGE_SetMotCurrentPhaseA(&spi_tx_buf, idx, phase_current);
			return;
		}
		if (msgHeader.StdId == (0x581 + idx)) {
			float phase_current = 0;
			memcpy(&phase_current, msgData, sizeof(float));
			PACKAGE_SetMotCurrentPhaseB(&spi_tx_buf, idx, phase_current);
			return;
		}
		if (msgHeader.StdId == (0x681 + idx)) {
			float phase_current = 0;
			memcpy(&phase_current, msgData, sizeof(float));
			PACKAGE_SetMotCurrentPhaseC(&spi_tx_buf, idx, phase_current);
			return;
		}
	}
}

void TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	
}

void TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	
}

void TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance == ADC1) {
    PACKAGE_SetCurrentLightLeft(&spi_tx_buf, (float)adcData[0]);
    PACKAGE_SetCurrentLightRight(&spi_tx_buf, (float)adcData[1]);
    float voltage = 4.1956e-06 * (float)adcData[2] * (float)adcData[2] - 0.0057 * (float)adcData[2] + 12.9609;
		PACKAGE_SetVoltage24V(&spi_tx_buf, voltage);
		PACKAGE_SetCurrentGeneral(&spi_tx_buf, (float)adcData[3]);
    adcConvCpltStatus = true;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PACKAGE_TelemetryPackageInit(&spi_tx_buf);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_CAN_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(RPI_5V_EN_GPIO_Port, RPI_5V_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MODEM_12V_EN_GPIO_Port, MODEM_12V_EN_Pin, GPIO_PIN_SET);
	//HAL_Delay(40000);

	LSM6_Init(&hlsm6, &hi2c1);
	LIS3MDL_Init(&hlis3mdl, &hi2c1);
	MINIMU9_Init(&hminimu9, &hlsm6, &hlis3mdl, 0.03f);
	
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

	for (uint8_t idx = 0; idx < 6; idx++) {
    PWM_MOTx_SetServo(idx, 0);
  }

	HAL_CAN_Start(&hcan);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (adcConvCpltStatus) {
      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcData, ADC_CHANNELS_NUM);
      adcConvCpltStatus = false;
    }

    MINIMU9_Fusion(&hminimu9);

		float euler[] = {
			(180.0f / 3.14f) * hminimu9.Euler.x,
			(180.0f / 3.14f) * hminimu9.Euler.y,
			(180.0f / 3.14f) * hminimu9.Euler.z
		};

		float accel[] = {
			hminimu9.Accel.x,
			hminimu9.Accel.y,
			hminimu9.Accel.z
		};

		float gyro[] = {
			hminimu9.Gyro.x,
			hminimu9.Gyro.y,
			hminimu9.Gyro.z
		};

		float magnet[] = {
			hminimu9.Mag.x,
			hminimu9.Mag.y,
			hminimu9.Mag.z
		};

		PACKAGE_SetIMUEuler(&spi_tx_buf, euler);
		PACKAGE_SetIMUAccel(&spi_tx_buf, accel);
		PACKAGE_SetIMUGyro(&spi_tx_buf, gyro);
		PACKAGE_SetIMUMagnet(&spi_tx_buf, magnet);

#if 1
    if (spiTxRxCpltStatus) {
      HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&spi_tx_buf, (uint8_t *)&spi_rx_buf, SPI_BUFFER_SIZE);
      if (PACKAGE_ControlPackageCheck(&spi_rx_buf) == HAL_OK) {
        float mot_ctrl[6];
        float cam_servo;
        float led[2];
        if (PACKAGE_GetDesMotCtrl(&spi_rx_buf, mot_ctrl) == HAL_OK) {
          for (uint8_t idx = 0; idx < 6; idx++) {
#ifndef FOC_DRIVERS_MODE
            PWM_MOTx_SetServo(idx, mot_ctrl[idx]);
#else
            FOC_MOTx_SetRPM(idx, mot_ctrl[idx]);
#endif
          }
        }

#ifdef HYBRID_ROV_MODE
        for (uint8_t unit = 0; unit < UWMANIPULATOR_UNITS; unit++) {
          float angle = 0.0f;
          if (PACKAGE_GetDesManQ(&spi_rx_buf, unit, &angle) == HAL_OK) {
            UWManipulator_SetPosition(&huwman, unit, angle);
          }
        }

        uint8_t state = 0;
        if (PACKAGE_GetDesManGripState(&spi_rx_buf, &state) == HAL_OK) {
          UWManipulator_SetGripState(&huwman, state);
        }
#endif

        if (PACKAGE_GetDesCamServo(&spi_rx_buf, &cam_servo) == HAL_OK) {
          //printf("CAM_SERVO: %.2f\r\n", cam_servo);
        }

        if (PACKAGE_GetDesLED(&spi_rx_buf, led) == HAL_OK) {
          for (uint8_t idx = 0; idx < 2; idx++) {
            //printf("LED[%d]: %.2f\r\n", idx, led[idx]);
          }
        }
      }
      spiTxRxCpltStatus = false;
    }
#endif

#if 0
    static int32_t xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
    LIS3MDL_Read_Mag(&hlis3mdl);
    if (xmin > hlis3mdl.Mag.x) xmin = hlis3mdl.Mag.x;
    if (xmax < hlis3mdl.Mag.x) xmax = hlis3mdl.Mag.x;
    if (ymin > hlis3mdl.Mag.y) ymin = hlis3mdl.Mag.y;
    if (ymax < hlis3mdl.Mag.y) ymax = hlis3mdl.Mag.y;
    if (zmin > hlis3mdl.Mag.z) zmin = hlis3mdl.Mag.z;
    if (zmax < hlis3mdl.Mag.z) zmax = hlis3mdl.Mag.z;
    printf("xmin: %d\n\r", xmin);
    printf("xmax: %d\n\r", xmax);
    printf("ymin: %d\n\r", ymin);
    printf("ymax: %d\n\r", ymax);
    printf("zmin: %d\n\r", zmin);
    printf("zmax: %d\n\r\n\r", zmax);
		HAL_Delay(100);
#endif

    //printf("x:%.2f,y:%.2f,z:%.2f\n\r", (180.0f / 3.14f) * hminimu9.Euler.x, (180.0f / 3.14f) * hminimu9.Euler.y, (180.0f / 3.14f) * hminimu9.Euler.z);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef sFilterConfig = {0};
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN_Init 2 */

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
  htim1.Init.Prescaler = 10;
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
  htim2.Init.Prescaler = 10;
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
  htim3.Init.Prescaler = 10;
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
  htim4.Init.Prescaler = 10;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, Dir1_Pin|Dir2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(En__GPIO_Port, En__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MODEM_12V_EN_Pin|RPI_5V_EN_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : MODEM_12V_EN_Pin RPI_5V_EN_Pin */
  GPIO_InitStruct.Pin = MODEM_12V_EN_Pin|RPI_5V_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int stdout_putchar(int ch)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
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
