#ifndef _LIS3MDL_H
#define _LIS3MDL_H

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "matrix.h"

#define WHO_AM_I    0x0F
#define CTRL_REG1   0x20
#define CTRL_REG2   0x21
#define CTRL_REG3   0x22
#define CTRL_REG4   0x23
#define CTRL_REG5   0x24
#define LIS3MDL_STATUS_REG  0x27
#define OUT_X_L     0x28
#define OUT_X_H     0x29
#define OUT_Y_L     0x2A
#define OUT_Y_H     0x2B
#define OUT_Z_L     0x2C
#define OUT_Z_H     0x2D
#define TEMP_OUT_L  0x2E
#define TEMP_OUT_H  0x2F
#define INT_CFG     0x30
#define INT_SRC     0x31
#define INT_THS_L   0x32
#define INT_THS_H   0x33

typedef struct {
  I2C_HandleTypeDef *hi2c;
  HAL_StatusTypeDef Status;
  uint8_t Address;
  Vector3dTypeDef Mag;
} LIS3MDL_HandleTypeDef;

HAL_StatusTypeDef LIS3MDL_Init(LIS3MDL_HandleTypeDef *hlis3mdl, I2C_HandleTypeDef *hi2c);

void LIS3MDL_Enable_Default(LIS3MDL_HandleTypeDef *hlis3mdl);

void LIS3MDL_Write_Reg(LIS3MDL_HandleTypeDef *hlis3mdl, uint8_t Reg, uint8_t Value);

uint8_t LIS3MDL_Read_Reg(LIS3MDL_HandleTypeDef *hlis3mdl, uint8_t Reg);

void LIS3MDL_Read_Mag(LIS3MDL_HandleTypeDef *hlis3mdl);

#endif
