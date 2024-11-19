#ifndef _LSM6DS33_H
#define _LSM6DS33_H

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "matrix.h"

#define LSM6DS33_FIFO_CTRL1 0x06
#define LSM6DS33_FIFO_CTRL2 0x07
#define LSM6DS33_FIFO_CTRL3 0x08
#define LSM6DS33_FIFO_CTRL4 0x09
#define LSM6DS33_FIFO_CTRL5 0x0A

#define LSM6DS33_ORIENT_CFG_G 0x0B
#define LSM6DS33_INT1_CTRL 0x0D
#define LSM6DS33_INT2_CTRL 0x0E
#define LSM6DS33_WHO_AM_I 0x0F
#define LSM6DS33_CTRL1_XL 0x10
#define LSM6DS33_CTRL2_G 0x11
#define LSM6DS33_CTRL3_C 0x12
#define LSM6DS33_CTRL4_C 0x13
#define LSM6DS33_CTRL5_C 0x14
#define LSM6DS33_CTRL6_C 0x15
#define LSM6DS33_CTRL7_G 0x16
#define LSM6DS33_CTRL8_XL 0x17
#define LSM6DS33_CTRL9_XL 0x18
#define LSM6DS33_CTRL10_C 0x19
#define LSM6DS33_STATUS_REG 0x1E

#define LSM6DS33_OUT_TEMP_L 0x20
#define LSM6DS33_OUT_TEMP_H 0x21
#define LSM6DS33_OUTX_L_G 0x22
#define LSM6DS33_OUTX_H_G 0x23
#define LSM6DS33_OUTY_L_G 0x24
#define LSM6DS33_OUTY_H_G 0x25
#define LSM6DS33_OUTZ_L_G 0x26
#define LSM6DS33_OUTZ_H_G 0x27
#define LSM6DS33_OUTX_L_XL 0x28
#define LSM6DS33_OUTX_H_XL 0x29
#define LSM6DS33_OUTY_L_XL 0x2A
#define LSM6DS33_OUTY_H_XL 0x2B
#define LSM6DS33_OUTZ_L_XL 0x2C
#define LSM6DS33_OUTZ_H_XL 0x2D

#define LSM6DS33_FIFO_STATUS3 0x3C
#define LSM6DS33_FIFO_STATUS4 0x3D
#define LSM6DS33_FIFO_DATA_OUT_L 0x3E
#define LSM6DS33_FIFO_DATA_OUT_H 0x3F
#define LSM6DS33_TIMESTAMP0_REG 0x40
#define LSM6DS33_TIMESTAMP1_REG 0x41
#define LSM6DS33_TIMESTAMP2_REG 0x42

#define LSM6DS33_STEP_TIMESTAMP_L 0x49
#define LSM6DS33_STEP_TIMESTAMP_H 0x4A
#define LSM6DS33_STEP_COUNTER_L 0x4B
#define LSM6DS33_STEP_COUNTER_H 0x4C

typedef struct {
  I2C_HandleTypeDef *hi2c;
  HAL_StatusTypeDef Status;
  uint8_t Address;
  Vector3dTypeDef RawAccel;
  Vector3dTypeDef RawGyro;
	Vector3dTypeDef RawGyroOffset;
} LSM6_HandleTypeDef;

HAL_StatusTypeDef LSM6_Init(LSM6_HandleTypeDef *hlsm6, I2C_HandleTypeDef *hi2c);

void LSM6_Enable_Default(LSM6_HandleTypeDef *hlsm6);

void LSM6_Write_Reg(LSM6_HandleTypeDef *hlsm6, uint8_t Reg, uint8_t Value);

uint8_t LSM6_Read_Reg(LSM6_HandleTypeDef *hlsm6, uint8_t Reg);

void LSM6_Read_RawAccel(LSM6_HandleTypeDef *hlsm6);

void LSM6_Read_RawGyro(LSM6_HandleTypeDef *hlsm6);

void LSM6_Measure_Offsets(LSM6_HandleTypeDef *hlsm6);

#endif
