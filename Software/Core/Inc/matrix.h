#ifndef _MATRIX_H
#define _MATRIX_H

#include <stdint.h>

typedef struct {
  float x, y, z;
} Vector3fTypeDef;

typedef struct {
  int16_t x, y, z;
} Vector3dTypeDef;

void LSM6_Vector_Cross(Vector3fTypeDef *a, Vector3fTypeDef *b,
                       Vector3fTypeDef *out);

float LSM6_Vector_Dot(Vector3fTypeDef *a, Vector3fTypeDef *b);

void LSM6_Vector_Normalize(Vector3fTypeDef *a);

#endif
