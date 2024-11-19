#include "lsm6ds33.h"

#include <math.h>
#include <limits.h>

#define LSM6DS33_SA0_HIGH_ADDRESS (0x006B << 1)
#define LSM6DS33_SA0_LOW_ADDRESS (0x006A << 1)

#define LSM6DS33_WHO_ID (0x69)

static HAL_StatusTypeDef LSM6_Test_Reg(LSM6_HandleTypeDef *hlsm6,
                                       uint16_t Address, uint8_t Reg,
                                       uint8_t *Out) {
	return HAL_I2C_Mem_Read(hlsm6->hi2c, Address, Reg, sizeof(uint8_t),
				                  Out, sizeof(uint8_t), 10);
}

HAL_StatusTypeDef LSM6_Init(LSM6_HandleTypeDef *hlsm6, I2C_HandleTypeDef *hi2c) {
  uint8_t Reg = 0;
	HAL_StatusTypeDef Status = HAL_OK;
	
	hlsm6->hi2c = hi2c;

	Status = LSM6_Test_Reg(hlsm6, LSM6DS33_SA0_HIGH_ADDRESS, LSM6DS33_WHO_AM_I, &Reg);
  if (Status == HAL_OK) {
    if (Reg == LSM6DS33_WHO_ID) {
      hlsm6->Address = LSM6DS33_SA0_HIGH_ADDRESS;
      return HAL_OK;
    }
  }

	Status = LSM6_Test_Reg(hlsm6, LSM6DS33_SA0_LOW_ADDRESS, LSM6DS33_WHO_AM_I, &Reg);
  if (Status == HAL_OK) {
    if (Reg == LSM6DS33_WHO_ID) {
      hlsm6->Address = LSM6DS33_SA0_LOW_ADDRESS;
      return HAL_OK;
    }
  }

  return HAL_ERROR;
}

void LSM6_Enable_Default(LSM6_HandleTypeDef *hlsm6) {
	LSM6_Write_Reg(hlsm6, LSM6DS33_CTRL2_G, 0x8C);
  LSM6_Write_Reg(hlsm6, LSM6DS33_CTRL7_G, 0x00);
  LSM6_Write_Reg(hlsm6, LSM6DS33_CTRL1_XL, 0x8C);
  LSM6_Write_Reg(hlsm6, LSM6DS33_CTRL3_C, 0x04);
}

void LSM6_Write_Reg(LSM6_HandleTypeDef *hlsm6, uint8_t Reg, uint8_t Value) {
  hlsm6->Status = HAL_I2C_Mem_Write(hlsm6->hi2c, hlsm6->Address, Reg, sizeof(uint8_t),
                                    &Value, sizeof(uint8_t), 1);
}

uint8_t LSM6_Read_Reg(LSM6_HandleTypeDef *hlsm6, uint8_t Reg) {
  uint8_t Value;

	hlsm6->Status = HAL_I2C_Mem_Read(hlsm6->hi2c, hlsm6->Address, Reg, sizeof(uint8_t),
                                   &Value, sizeof(Value), 1);

  return Value;
}

void LSM6_Read_RawAccel(LSM6_HandleTypeDef *hlsm6) {
  uint8_t Data[6] = {0};

	hlsm6->Status = HAL_I2C_Mem_Read(hlsm6->hi2c, hlsm6->Address, LSM6DS33_OUTX_L_XL,
	                                 sizeof(uint8_t), Data, sizeof(Data), 1);

  uint8_t xla = Data[0];
  uint8_t xha = Data[1];
  uint8_t yla = Data[2];
  uint8_t yha = Data[3];
  uint8_t zla = Data[4];
  uint8_t zha = Data[5];

  hlsm6->RawAccel.x = (int16_t)(xha << CHAR_BIT | xla);
  hlsm6->RawAccel.y = (int16_t)(yha << CHAR_BIT | yla);
  hlsm6->RawAccel.z = (int16_t)(zha << CHAR_BIT | zla);
}

void LSM6_Read_RawGyro(LSM6_HandleTypeDef *hlsm6) {
  uint8_t Data[6] = {0};

	hlsm6->Status = HAL_I2C_Mem_Read(hlsm6->hi2c, hlsm6->Address, LSM6DS33_OUTX_L_G,
	                                 sizeof(uint8_t), Data, sizeof(Data), 1);

  uint8_t xlg = Data[0];
  uint8_t xhg = Data[1];
  uint8_t ylg = Data[2];
  uint8_t yhg = Data[3];
  uint8_t zlg = Data[4];
  uint8_t zhg = Data[5];

  hlsm6->RawGyro.x = (int16_t)(xhg << CHAR_BIT | xlg);
  hlsm6->RawGyro.y = (int16_t)(yhg << CHAR_BIT | ylg);
  hlsm6->RawGyro.z = (int16_t)(zhg << CHAR_BIT | zlg);
}

void LSM6_Measure_Offsets(LSM6_HandleTypeDef *hlsm6) {
  int32_t GyroOffsetX = 0;
	int32_t GyroOffsetY = 0;
	int32_t GyroOffsetZ = 0;

  size_t sampleCount = 32;
  for (size_t i = 0; i < sampleCount; i++) {
    LSM6_Read_RawGyro(hlsm6);
    GyroOffsetX += hlsm6->RawGyro.x;
	  GyroOffsetY += hlsm6->RawGyro.y;
	  GyroOffsetZ += hlsm6->RawGyro.z;
    HAL_Delay(20);
  }

  hlsm6->RawGyroOffset.x = GyroOffsetX / sampleCount;
	hlsm6->RawGyroOffset.y = GyroOffsetY / sampleCount;
	hlsm6->RawGyroOffset.z = GyroOffsetZ / sampleCount;
}
