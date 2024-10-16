#include "lsm6ds33.h"

#include <math.h>
#include <stdio.h>

#define DS33_SA0_HIGH_ADDRESS (0x006B << 1)
#define DS33_SA0_LOW_ADDRESS (0x006A << 1)

#define DS33_WHO_ID (0x69)

static HAL_StatusTypeDef LSM6_Test_Reg(LSM6_HandleTypeDef *hlsm6,
                                       uint16_t Address, uint8_t Reg,
                                       uint8_t *Out) {
  // TODO ((uint16_t)Address << 1)
	uint8_t Data[] = { Reg };

  if (HAL_I2C_Master_Transmit(hlsm6->hi2c, Address, Data, sizeof(Data), 10) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_I2C_Master_Receive(hlsm6->hi2c, Address, Out, 1, 10) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef LSM6_Init(LSM6_HandleTypeDef *hlsm6, I2C_HandleTypeDef *hi2c) {
  uint8_t Reg = 0;
	HAL_StatusTypeDef Status = HAL_OK;
	
	hlsm6->hi2c = hi2c;
	
	printf("%s\n", __func__);

	Status = LSM6_Test_Reg(hlsm6, DS33_SA0_HIGH_ADDRESS, WHO_AM_I, &Reg);
  if (Status == HAL_OK) {
		printf("ADDR %X: OK\n", DS33_SA0_HIGH_ADDRESS);
    if (Reg == DS33_WHO_ID) {
      hlsm6->Address = DS33_SA0_HIGH_ADDRESS;
      return HAL_OK;
    }
  }
	printf("ADDR %X: FAIL\n", DS33_SA0_HIGH_ADDRESS);

	Status = LSM6_Test_Reg(hlsm6, DS33_SA0_LOW_ADDRESS, WHO_AM_I, &Reg);
  if (Status == HAL_OK) {
		printf("ADDR %X: OK\n", DS33_SA0_LOW_ADDRESS);
    if (Reg == DS33_WHO_ID) {
      hlsm6->Address = DS33_SA0_LOW_ADDRESS;
      return HAL_OK;
    }
  }
	printf("ADDR %X: FAIL\n", DS33_SA0_LOW_ADDRESS);

  return HAL_ERROR;
}

void LSM6_Enable_Default(LSM6_HandleTypeDef *hlsm6) {
  LSM6_Write_Reg(hlsm6, CTRL1_XL, 0x80);
  LSM6_Write_Reg(hlsm6, CTRL2_G, 0x80);
  LSM6_Write_Reg(hlsm6, CTRL3_C, 0x04);
}

void LSM6_Write_Reg(LSM6_HandleTypeDef *hlsm6, uint8_t Reg, uint8_t Value) {
  uint8_t Data[] = { Reg, Value };
  hlsm6->Status = HAL_I2C_Master_Transmit(hlsm6->hi2c, hlsm6->Address,
                                          Data, sizeof(Data), HAL_MAX_DELAY);
}

uint8_t LSM6_Read_Reg(LSM6_HandleTypeDef *hlsm6, uint8_t Reg) {
  uint8_t Value;
	uint8_t Data[] = { Reg };

  hlsm6->Status = HAL_I2C_Master_Transmit(hlsm6->hi2c, hlsm6->Address,
                                          Data, sizeof(Data), HAL_MAX_DELAY);

  hlsm6->Status = HAL_I2C_Master_Receive(hlsm6->hi2c, hlsm6->Address,
                                         &Value, 1, HAL_MAX_DELAY);

  return Value;
}

void LSM6_Read_Acc(LSM6_HandleTypeDef *hlsm6) {
  uint8_t Data[6] = { 0 };

	uint8_t Buf[] = { OUTX_L_XL };
  hlsm6->Status = HAL_I2C_Master_Transmit(hlsm6->hi2c, hlsm6->Address,
                                          Buf, sizeof(Buf), HAL_MAX_DELAY);

  hlsm6->Status = HAL_I2C_Master_Receive(hlsm6->hi2c, hlsm6->Address,
                                         Data, sizeof(Data), HAL_MAX_DELAY);

  uint8_t xla = Data[0];
  uint8_t xha = Data[1];
  uint8_t yla = Data[2];
  uint8_t yha = Data[3];
  uint8_t zla = Data[4];
  uint8_t zha = Data[5];

  hlsm6->Accel.x = (int16_t)(xha << 8 | xla);
  hlsm6->Accel.y = (int16_t)(yha << 8 | yla);
  hlsm6->Accel.z = (int16_t)(zha << 8 | zla);
}

void LSM6_Read_Gyro(LSM6_HandleTypeDef *hlsm6) {
  uint8_t Data[6] = { 0 };

	uint8_t Buf[] = { OUTX_L_G };
  hlsm6->Status = HAL_I2C_Master_Transmit(hlsm6->hi2c, hlsm6->Address,
                                          Buf, sizeof(Buf), HAL_MAX_DELAY);

  hlsm6->Status = HAL_I2C_Master_Receive(hlsm6->hi2c, hlsm6->Address,
                                         Data, sizeof(Data), HAL_MAX_DELAY);

  uint8_t xlg = Data[0];
  uint8_t xhg = Data[1];
  uint8_t ylg = Data[2];
  uint8_t yhg = Data[3];
  uint8_t zlg = Data[4];
  uint8_t zhg = Data[5];

  hlsm6->Gyro.x = (int16_t)(xhg << 8 | xlg);
  hlsm6->Gyro.y = (int16_t)(yhg << 8 | ylg);
  hlsm6->Gyro.z = (int16_t)(zhg << 8 | zlg);
}

void LSM6_Read(LSM6_HandleTypeDef *hlsm6) {
  LSM6_Read_Acc(hlsm6);
  LSM6_Read_Gyro(hlsm6);
}
