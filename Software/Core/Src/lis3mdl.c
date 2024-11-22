#include "lis3mdl.h"

#include <math.h>
#include <limits.h>

#define LIS3MDL_SA1_HIGH_ADDRESS (0x001E << 1)
#define LIS3MDL_SA1_LOW_ADDRESS (0x001C << 1)

#define LIS3MDL_WHO_ID (0x3D)

static HAL_StatusTypeDef LIS3MDL_Test_Reg(LIS3MDL_HandleTypeDef *hlis3mdl,
                                          uint16_t Address, uint8_t Reg,
                                          uint8_t *Out) {
  return HAL_I2C_Mem_Read(hlis3mdl->hi2c, Address, Reg, sizeof(uint8_t),
                          Out, sizeof(uint8_t), 10);
}

HAL_StatusTypeDef LIS3MDL_Init(LIS3MDL_HandleTypeDef *hlis3mdl, I2C_HandleTypeDef *hi2c) {
  uint8_t Reg = 0;
  HAL_StatusTypeDef Status = HAL_OK;

  hlis3mdl->hi2c = hi2c;

  Status = LIS3MDL_Test_Reg(hlis3mdl, LIS3MDL_SA1_HIGH_ADDRESS, LIS3MDL_WHO_AM_I, &Reg);
  if (Status == HAL_OK) {
    if (Reg == LIS3MDL_WHO_ID) {
      hlis3mdl->Address = LIS3MDL_SA1_HIGH_ADDRESS;
      return HAL_OK;
    }
  }

  Status = LIS3MDL_Test_Reg(hlis3mdl, LIS3MDL_SA1_LOW_ADDRESS, LIS3MDL_WHO_AM_I, &Reg);
  if (Status == HAL_OK) {
    if (Reg == LIS3MDL_WHO_ID) {
      hlis3mdl->Address = LIS3MDL_SA1_LOW_ADDRESS;
      return HAL_OK;
    }
  }

  return HAL_ERROR;
}

void LIS3MDL_Enable_Default(LIS3MDL_HandleTypeDef *hlis3mdl) {
  LIS3MDL_Write_Reg(hlis3mdl, LIS3MDL_CTRL_REG1, 0x70);
  LIS3MDL_Write_Reg(hlis3mdl, LIS3MDL_CTRL_REG2, 0x00);
  LIS3MDL_Write_Reg(hlis3mdl, LIS3MDL_CTRL_REG3, 0x00);
  LIS3MDL_Write_Reg(hlis3mdl, LIS3MDL_CTRL_REG4, 0x0C);
}

void LIS3MDL_Write_Reg(LIS3MDL_HandleTypeDef *hlis3mdl, uint8_t Reg, uint8_t Value) {
  hlis3mdl->Status = HAL_I2C_Mem_Write(hlis3mdl->hi2c, hlis3mdl->Address, Reg, sizeof(uint8_t),
                                       &Value, sizeof(uint8_t), 1);
}

uint8_t LIS3MDL_Read_Reg(LIS3MDL_HandleTypeDef *hlis3mdl, uint8_t Reg) {
  uint8_t Value;

  hlis3mdl->Status = HAL_I2C_Mem_Read(hlis3mdl->hi2c, hlis3mdl->Address, Reg, sizeof(uint8_t),
                                      &Value, sizeof(Value), 1);

  return Value;
}

void LIS3MDL_Read_RawMag(LIS3MDL_HandleTypeDef *hlis3mdl) {
  uint8_t Data[6] = {0};

  hlis3mdl->Status = HAL_I2C_Mem_Read(hlis3mdl->hi2c, hlis3mdl->Address, (0x80 | LIS3MDL_OUT_X_L),
                                      sizeof(uint8_t), Data, sizeof(Data), 1);

  uint8_t xlm = Data[0];
  uint8_t xhm = Data[1];
  uint8_t ylm = Data[2];
  uint8_t yhm = Data[3];
  uint8_t zlm = Data[4];
  uint8_t zhm = Data[5];

  hlis3mdl->RawMag.x = (int16_t)(xhm << CHAR_BIT | xlm);
  hlis3mdl->RawMag.y = (int16_t)(yhm << CHAR_BIT | ylm);
  hlis3mdl->RawMag.z = (int16_t)(zhm << CHAR_BIT | zlm);
}
