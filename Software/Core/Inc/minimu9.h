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
	Vector3fTypeDef Euler;
	QuaternionTypeDef Rotation;

	float dt;
	float time;
} MINIMU9_HandleTypeDef;

HAL_StatusTypeDef MINIMU9_Init(MINIMU9_HandleTypeDef *hminimu9, LSM6_HandleTypeDef *hlsm6, LIS3MDL_HandleTypeDef *hlis3mdl, float dt);

HAL_StatusTypeDef MINIMU9_Read_Accel(MINIMU9_HandleTypeDef *hminimu9);

HAL_StatusTypeDef MINIMU9_Read_Gyro(MINIMU9_HandleTypeDef *hminimu9);

HAL_StatusTypeDef MINIMU9_Read_Mag(MINIMU9_HandleTypeDef *hminimu9);

void MINIMU9_Fusion(MINIMU9_HandleTypeDef *hminimu9);

void MINIMU9_MadgwickCycle(MINIMU9_HandleTypeDef *hminimu9, Vector3fTypeDef *a, Vector3fTypeDef *om, Vector3fTypeDef *mfield, float dt);

#endif
