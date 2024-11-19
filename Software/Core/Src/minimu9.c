#include "minimu9.h"

#include <math.h>
#include <stdio.h>

HAL_StatusTypeDef MINIMU9_Init(MINIMU9_HandleTypeDef *hminimu9, LSM6_HandleTypeDef *hlsm6, LIS3MDL_HandleTypeDef *hlis3mdl, float dt) {
	hminimu9->hlsm6 = hlsm6;
	hminimu9->hlis3mdl = hlis3mdl;
	hminimu9->dt = dt;
	hminimu9->time = 0.0f;
	Quaternion_Identity(&hminimu9->Rotation);

	LSM6_Enable_Default(hminimu9->hlsm6);
	LSM6_Measure_Offsets(hminimu9->hlsm6);
	LIS3MDL_Enable_Default(hminimu9->hlis3mdl);

  return HAL_OK;
}

HAL_StatusTypeDef MINIMU9_Read_Accel(MINIMU9_HandleTypeDef *hminimu9) {
	LSM6_Read_RawAccel(hminimu9->hlsm6);

	hminimu9->Accel.x = hminimu9->hlsm6->RawAccel.x * 0.000244;
	hminimu9->Accel.y = hminimu9->hlsm6->RawAccel.y * 0.000244;
	hminimu9->Accel.z = hminimu9->hlsm6->RawAccel.z * 0.000244;

	return HAL_OK;
}

HAL_StatusTypeDef MINIMU9_Read_Gyro(MINIMU9_HandleTypeDef *hminimu9) {
	LSM6_Read_RawGyro(hminimu9->hlsm6);

	hminimu9->Gyro.x = (hminimu9->hlsm6->RawGyro.x - hminimu9->hlsm6->RawGyroOffset.x) * (0.07 * 3.14159265 / 180);
	hminimu9->Gyro.y = (hminimu9->hlsm6->RawGyro.y - hminimu9->hlsm6->RawGyroOffset.y) * (0.07 * 3.14159265 / 180);
	hminimu9->Gyro.z = (hminimu9->hlsm6->RawGyro.z - hminimu9->hlsm6->RawGyroOffset.z) * (0.07 * 3.14159265 / 180);

	return HAL_OK;
}

HAL_StatusTypeDef MINIMU9_Read_Mag(MINIMU9_HandleTypeDef *hminimu9) {
	LIS3MDL_Read_RawMag(hminimu9->hlis3mdl);
	Vector3dTypeDef mag_min = { -4670, -2584, -3220 };
	Vector3dTypeDef mag_max = { 893, 3959, 2702 };

	hminimu9->Mag.x = (float)(hminimu9->hlis3mdl->RawMag.x - mag_min.x) / (mag_max.x - mag_min.x) * 2 - 1;
  hminimu9->Mag.y = (float)(hminimu9->hlis3mdl->RawMag.y - mag_min.y) / (mag_max.y - mag_min.y) * 2 - 1;
  hminimu9->Mag.z = (float)(hminimu9->hlis3mdl->RawMag.z - mag_min.z) / (mag_max.z - mag_min.z) * 2 - 1;

	return HAL_OK;
}

static void Rotation_From_Compass(Vector3fTypeDef *a, Vector3fTypeDef *mfield, MatrixRotationTypeDef Rot) {
  Vector3fTypeDef down = { -a->x, -a->y, -a->z };
  Vector3fTypeDef east = { 0, 0, 0 };
	Vector3f_Cross(&down, mfield, &east);
  Vector3fTypeDef north = { 0, 0, 0 };
	Vector3f_Cross(&east, &down, &north);

	Vector3f_Normalize(&east);
	Vector3f_Normalize(&north);
	Vector3f_Normalize(&down);

	Rot[0][0] = north.x;
	Rot[0][1] = north.y;
	Rot[0][2] = north.z;

	Rot[1][0] = east.x;
	Rot[1][1] = east.y;
	Rot[1][2] = east.z;

	Rot[2][0] = down.x;
	Rot[2][1] = down.y;
	Rot[2][2] = down.z;
}

static void Rotate(QuaternionTypeDef *q, Vector3fTypeDef *om, float dt) {
	QuaternionTypeDef q2 = {
		.x = om->x * dt / 2.0f,
		.y = om->y * dt / 2.0f,
		.z = om->z * dt / 2.0f,
		.w = 1.0f
	};
	QuaternionTypeDef q1 = {
		.x = q->x,
		.y = q->y,
		.z = q->z,
		.w = q->w
	};
	Quaternion_Multiply(&q1, &q2, q);
  Quaternion_Normalize(q);
}

void MINIMU9_Fusion(MINIMU9_HandleTypeDef *hminimu9) {
	float time = HAL_GetTick() / 1000.0f;
	float dt = time - hminimu9->time;

	if (dt < hminimu9->dt) return;
	hminimu9->time = time;

	MINIMU9_Read_Accel(hminimu9);
	MINIMU9_Read_Gyro(hminimu9);
	MINIMU9_Read_Mag(hminimu9);

	Vector3fTypeDef a = hminimu9->Accel;
	Vector3fTypeDef om = hminimu9->Gyro;
	Vector3fTypeDef mfield = hminimu9->Mag;

  Vector3fTypeDef Correction = { 0.0f, 0.0f, 0.0f };

  if (fabs(Vector3f_Norm(&a) - 1.0f) <= 0.3f) {
    float CorrectionStrength = 1.0f;

		MatrixRotationTypeDef RotCompass = { 0 };
		Rotation_From_Compass(&a, &mfield, RotCompass);
		MatrixRotationTypeDef Rot = { 0 };
		Quaternion_To_Matrix(&hminimu9->Rotation, Rot);

		Vector3fTypeDef RotCompass_Row1 = { RotCompass[0][0], RotCompass[0][1], RotCompass[0][2] };
		Vector3fTypeDef RotCompass_Row2 = { RotCompass[1][0], RotCompass[1][1], RotCompass[1][2] };
		Vector3fTypeDef RotCompass_Row3 = { RotCompass[2][0], RotCompass[2][1], RotCompass[2][2] };

		Vector3fTypeDef Rot_Row1 = { Rot[0][0], Rot[0][1], Rot[0][2] };
		Vector3fTypeDef Rot_Row2 = { Rot[1][0], Rot[1][1], Rot[1][2] };
		Vector3fTypeDef Rot_Row3 = { Rot[2][0], Rot[2][1], Rot[2][2] };

		Vector3fTypeDef Correction1 = { 0.0f, 0.0f, 0.0f };
		Vector3fTypeDef Correction2 = { 0.0f, 0.0f, 0.0f };
		Vector3fTypeDef Correction3 = { 0.0f, 0.0f, 0.0f };
		
		Vector3f_Cross(&RotCompass_Row1, &Rot_Row1, &Correction1);
		Vector3f_Cross(&RotCompass_Row2, &Rot_Row2, &Correction2);
		Vector3f_Cross(&RotCompass_Row3, &Rot_Row3, &Correction2);

		Correction.x = Correction1.x + Correction2.x + Correction3.x;
		Correction.y = Correction1.y + Correction2.y + Correction3.y;
		Correction.z = Correction1.z + Correction2.z + Correction3.z;
		
		Correction.x *= CorrectionStrength;
		Correction.y *= CorrectionStrength;
		Correction.z *= CorrectionStrength;
  }

	Vector3fTypeDef CorrectedOm = {
		om.x + Correction.x,
		om.y + Correction.y,
		om.z + Correction.z
	};
  Rotate(&hminimu9->Rotation, &CorrectedOm, dt);
	Quaternion_To_Euler(&hminimu9->Rotation, &hminimu9->Euler);
}
