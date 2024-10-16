#ifndef _MATRIX_H
#define _MATRIX_H

#include <stdint.h>

typedef struct {
  float x, y, z;
} Vector3fTypeDef;

typedef struct {
  int16_t x, y, z;
} Vector3dTypeDef;

typedef struct {
  float x, y, z, w;
} Quaternion3fTypeDef;

typedef struct {
  int16_t x, y, z, w;
} Quaternion3dTypeDef;

void Vector3f_Cross(Vector3fTypeDef *a, Vector3fTypeDef *b, Vector3fTypeDef *out);

float Vector3f_Dot(Vector3fTypeDef *a, Vector3fTypeDef *b);

void Vector3f_Normalize(Vector3fTypeDef *a);

float Vector3d_Max(Vector3dTypeDef *a);

float Vector3d_Min(Vector3dTypeDef *a);

#endif
