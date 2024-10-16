#ifndef _MINIMU9_H
#define _MINIMU9_H

#include <stdint.h>

#include "lsm6ds33.h"
#include "lis3mdl.h"

typedef struct {
  LSM6_HandleTypeDef *hlsm6;
	LIS3MDL_HandleTypeDef *hlis3mdl;
	Vector3fTypeDef Accel;
	Vector3fTypeDef Gyro;
	Vector3fTypeDef Mag;
} MINIMU9_HandleTypeDef;

HAL_StatusTypeDef MINIMU9_Init(MINIMU9_HandleTypeDef *hminimu9, LSM6_HandleTypeDef *hlsm6, LIS3MDL_HandleTypeDef *hlis3mdl);

HAL_StatusTypeDef MINIMU9_Read_Accel(MINIMU9_HandleTypeDef *hminimu9);

HAL_StatusTypeDef MINIMU9_Read_Gyro(MINIMU9_HandleTypeDef *hminimu9);

HAL_StatusTypeDef MINIMU9_Read_Mag(MINIMU9_HandleTypeDef *hminimu9);

#endif
