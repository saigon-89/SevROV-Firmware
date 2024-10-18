#ifndef _LSM6_H
#define _LSM6_H

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "matrix.h"

#define FUNC_CFG_ACCESS 0x01

#define DS33_FIFO_CTRL1 0x06  // DS33
#define DS33_FIFO_CTRL2 0x07  // DS33
#define DS33_FIFO_CTRL3 0x08  // DS33
#define DS33_FIFO_CTRL4 0x09  // DS33
#define DS33_FIFO_CTRL5 0x0A  // DS33

#define ORIENT_CFG_G 0x0B      // DS33
#define INT1_CTRL 0x0D
#define INT2_CTRL 0x0E
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define CTRL4_C 0x13
#define CTRL5_C 0x14
#define CTRL6_C 0x15
#define CTRL7_G 0x16
#define CTRL8_XL 0x17
#define CTRL9_XL 0x18
#define CTRL10_C 0x19
#define WAKE_UP_SRC 0x1B
#define TAP_SRC 0x1C
#define D6D_SRC 0x1D
#define STATUS_REG 0x1E

#define OUT_TEMP_L 0x20
#define OUT_TEMP_H 0x21
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27
#define OUTX_L_XL 0x28  // DS33
#define OUTX_H_XL 0x29  // DS33
#define OUTY_L_XL 0x2A  // DS33
#define OUTY_H_XL 0x2B  // DS33
#define OUTZ_L_XL 0x2C  // DS33
#define OUTZ_H_XL 0x2D  // DS33

#define FIFO_STATUS1 0x3A
#define FIFO_STATUS2 0x3B
#define FIFO_STATUS3 0x3C     // DS33
#define FIFO_STATUS4 0x3D     // DS33
#define FIFO_DATA_OUT_L 0x3E  // DS33
#define FIFO_DATA_OUT_H 0x3F  // DS33
#define TIMESTAMP0_REG 0x40   // DS33
#define TIMESTAMP1_REG 0x41   // DS33
#define TIMESTAMP2_REG 0x42   // DS33

#define STEP_TIMESTAMP_L 0x49  // DS33
#define STEP_TIMESTAMP_H 0x4A  // DS33
#define DS33_STEP_COUNTER_L 0x4B  // DS33 (DSO version in embedded functions regs)
#define DS33_STEP_COUNTER_H 0x4C  // DS33

#define FUNC_SRC = 0x53,  // DS33

#define TAP_CFG 0x58   // DS33
#define TAP_THS_6D 0x59
#define INT_DUR2 0x5A
#define WAKE_UP_THS 0x5B
#define WAKE_UP_DUR 0x5C
#define FREE_FALL 0x5D
#define MD1_CFG 0x5E
#define MD2_CFG 0x5F

typedef struct {
  I2C_HandleTypeDef *hi2c;
  HAL_StatusTypeDef Status;
  uint8_t Address;
  Vector3dTypeDef Accel;
  Vector3dTypeDef Gyro;
	Vector3dTypeDef GyroOffset;
} LSM6_HandleTypeDef;

HAL_StatusTypeDef LSM6_Init(LSM6_HandleTypeDef *hlsm6, I2C_HandleTypeDef *hi2c);

void LSM6_Enable_Default(LSM6_HandleTypeDef *hlsm6);

void LSM6_Write_Reg(LSM6_HandleTypeDef *hlsm6, uint8_t Reg, uint8_t Value);

uint8_t LSM6_Read_Reg(LSM6_HandleTypeDef *hlsm6, uint8_t Reg);

void LSM6_Read_Acc(LSM6_HandleTypeDef *hlsm6);

void LSM6_Read_Gyro(LSM6_HandleTypeDef *hlsm6);

void LSM6_Read(LSM6_HandleTypeDef *hlsm6);

void LSM6_Measure_Offsets(LSM6_HandleTypeDef *hlsm6);

#endif
