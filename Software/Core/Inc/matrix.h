#ifndef _MATRIX_H
#define _MATRIX_H

#include <stdint.h>

typedef struct {
  float x, y, z;
} Vector3fTypeDef;

typedef struct {
  int16_t x, y, z;
} Vector3dTypeDef;

typedef float MatrixRotationTypeDef[3][3];

typedef struct {
  float x, y, z, w;
} QuaternionTypeDef;

void Quaternion_To_Matrix(QuaternionTypeDef *q, MatrixRotationTypeDef m);

void Quaternion_Multiply(QuaternionTypeDef *q1, QuaternionTypeDef *q2, QuaternionTypeDef *out);

void Quaternion_Normalize(QuaternionTypeDef *q);

void Quaternion_To_Euler(QuaternionTypeDef *q, Vector3fTypeDef *euler);

void Quaternion_Identity(QuaternionTypeDef *q);

void Vector3f_Cross(Vector3fTypeDef *a, Vector3fTypeDef *b, Vector3fTypeDef *out);

float Vector3f_Dot(Vector3fTypeDef *a, Vector3fTypeDef *b);

void Vector3f_Normalize(Vector3fTypeDef *a);

float Vector3f_Norm(Vector3fTypeDef *a);

float Vector3d_Max(Vector3dTypeDef *a);

float Vector3d_Min(Vector3dTypeDef *a);

#endif
