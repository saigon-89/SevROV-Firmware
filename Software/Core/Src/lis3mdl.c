#include "lis3mdl.h"

#include <math.h>
#include <stdio.h>

#define LIS3MDL_SA1_HIGH_ADDRESS (0x001E << 1)
#define LIS3MDL_SA1_LOW_ADDRESS (0x001C << 1)

#define LIS3MDL_WHO_ID (0x3D)

static HAL_StatusTypeDef LIS3MDL_Test_Reg(LIS3MDL_HandleTypeDef *hlis3mdl,
                                          uint16_t Address, uint8_t Reg,
                                          uint8_t *Out) {
	uint8_t Data[] = { Reg };

  if (HAL_I2C_Master_Transmit(hlis3mdl->hi2c, Address, Data, sizeof(Data), 10) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_I2C_Master_Receive(hlis3mdl->hi2c, Address, Out, 1, 10) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef LIS3MDL_Init(LIS3MDL_HandleTypeDef *hlis3mdl, I2C_HandleTypeDef *hi2c) {
  uint8_t Reg = 0;
	HAL_StatusTypeDef Status = HAL_OK;

	hlis3mdl->hi2c = hi2c;

	printf("%s\n", __func__);

	Status = LIS3MDL_Test_Reg(hlis3mdl, LIS3MDL_SA1_HIGH_ADDRESS, WHO_AM_I, &Reg);
  if (Status == HAL_OK) {
		printf("ADDR %X: OK\n", LIS3MDL_SA1_HIGH_ADDRESS);
    if (Reg == LIS3MDL_WHO_ID) {
      hlis3mdl->Address = LIS3MDL_SA1_HIGH_ADDRESS;
      return HAL_OK;
    }
  }
	printf("ADDR %X: FAIL\n", LIS3MDL_SA1_LOW_ADDRESS);

	Status = LIS3MDL_Test_Reg(hlis3mdl, LIS3MDL_SA1_LOW_ADDRESS, WHO_AM_I, &Reg);
  if (Status == HAL_OK) {
		printf("ADDR %X: OK\n", LIS3MDL_SA1_LOW_ADDRESS);
    if (Reg == LIS3MDL_WHO_ID) {
      hlis3mdl->Address = LIS3MDL_SA1_LOW_ADDRESS;
      return HAL_OK;
    }
  }
	printf("ADDR %X: FAIL\n", LIS3MDL_SA1_LOW_ADDRESS);

  return HAL_ERROR;
}

void LIS3MDL_Enable_Default(LIS3MDL_HandleTypeDef *hlis3mdl) {
#if 0
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG1, 0x70);
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG2, 0x00);
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG3, 0x00);
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG4, 0x0C);
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG5, 0x40);
#else
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG1, 0x70);
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG2, 0x00);
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG3, 0x00);
  LIS3MDL_Write_Reg(hlis3mdl, CTRL_REG4, 0x0C);
#endif
}

void LIS3MDL_Write_Reg(LIS3MDL_HandleTypeDef *hlis3mdl, uint8_t Reg, uint8_t Value) {
  uint8_t Data[] = { Reg, Value };
  hlis3mdl->Status = HAL_I2C_Master_Transmit(hlis3mdl->hi2c, hlis3mdl->Address,
                                             Data, sizeof(Data), HAL_MAX_DELAY);
}

uint8_t LIS3MDL_Read_Reg(LIS3MDL_HandleTypeDef *hlis3mdl, uint8_t Reg) {
  uint8_t Value;
	uint8_t Data[] = { Reg };

  hlis3mdl->Status = HAL_I2C_Master_Transmit(hlis3mdl->hi2c, hlis3mdl->Address,
                                             Data, sizeof(Data), HAL_MAX_DELAY);

  hlis3mdl->Status = HAL_I2C_Master_Receive(hlis3mdl->hi2c, hlis3mdl->Address,
                                            &Value, 1, HAL_MAX_DELAY);

  return Value;
}

void LIS3MDL_Read_Mag(LIS3MDL_HandleTypeDef *hlis3mdl) {
  uint8_t Data[6] = { 0 };

#if 0
	uint8_t Buf[] = { OUT_X_L };
#else
	uint8_t Buf[] = { 0x80 | OUT_X_L };
#endif

  hlis3mdl->Status = HAL_I2C_Master_Transmit(hlis3mdl->hi2c, hlis3mdl->Address,
                                             Buf, sizeof(Buf), HAL_MAX_DELAY);

  hlis3mdl->Status = HAL_I2C_Master_Receive(hlis3mdl->hi2c, hlis3mdl->Address,
                                            Data, sizeof(Data), HAL_MAX_DELAY);

  uint8_t xlm = Data[0];
  uint8_t xhm = Data[1];
  uint8_t ylm = Data[2];
  uint8_t yhm = Data[3];
  uint8_t zlm = Data[4];
  uint8_t zhm = Data[5];

  hlis3mdl->Mag.x = (int16_t)(xhm << 8 | xlm);
  hlis3mdl->Mag.y = (int16_t)(yhm << 8 | ylm);
  hlis3mdl->Mag.z = (int16_t)(zhm << 8 | zlm);
}
