#include "minimu9.h"

#include <math.h>
#include <stdio.h>

HAL_StatusTypeDef MINIMU9_Init(MINIMU9_HandleTypeDef *hminimu9, LSM6_HandleTypeDef *hlsm6, LIS3MDL_HandleTypeDef *hlis3mdl) {
	hminimu9->hlsm6 = hlsm6;
	hminimu9->hlis3mdl = hlis3mdl;
  return HAL_OK;
}

HAL_StatusTypeDef MINIMU9_Read_Accel(MINIMU9_HandleTypeDef *hminimu9) {
	LSM6_Read_Acc(hminimu9->hlsm6);
	hminimu9->Accel.x = hminimu9->hlsm6->Accel.x * 0.000244;
	hminimu9->Accel.y = hminimu9->hlsm6->Accel.y * 0.000244;
	hminimu9->Accel.z = hminimu9->hlsm6->Accel.z * 0.000244;
	return HAL_OK;
}

HAL_StatusTypeDef MINIMU9_Read_Gyro(MINIMU9_HandleTypeDef *hminimu9) {
	LSM6_Read_Gyro(hminimu9->hlsm6);
	hminimu9->Gyro.x = hminimu9->hlsm6->Gyro.x * (0.07 * 3.14159265 / 180);
	hminimu9->Gyro.y = hminimu9->hlsm6->Gyro.y * (0.07 * 3.14159265 / 180);
	hminimu9->Gyro.z = hminimu9->hlsm6->Gyro.z * (0.07 * 3.14159265 / 180);
	return HAL_OK;
}

HAL_StatusTypeDef MINIMU9_Read_Mag(MINIMU9_HandleTypeDef *hminimu9) {
	LIS3MDL_Read_Mag(hminimu9->hlis3mdl);
	Vector3dTypeDef mag_min = { 0, 0, 0 }; // from file
	Vector3dTypeDef mag_max = { 0, 0, 0 }; // from file
	hminimu9->Mag.x = (float)(hminimu9->hlis3mdl->Mag.x - mag_min.x) / (mag_max.x - mag_min.x) * 2 - 1;
  hminimu9->Mag.y = (float)(hminimu9->hlis3mdl->Mag.y - mag_min.y) / (mag_max.y - mag_min.y) * 2 - 1;
  hminimu9->Mag.z = (float)(hminimu9->hlis3mdl->Mag.z - mag_min.z) / (mag_max.z - mag_min.z) * 2 - 1;
	return HAL_OK;
}
