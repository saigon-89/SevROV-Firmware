#include "matrix.h"

#include <math.h>
#include <stdio.h>

void LSM6_Vector_Normalize(Vector3fTypeDef *a) {
  float mag = sqrt(LSM6_Vector_Dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

float LSM6_Vector_Dot(Vector3fTypeDef *a, Vector3fTypeDef *b) {
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void LSM6_Vector_Cross(Vector3fTypeDef *a, Vector3fTypeDef *b,
                       Vector3fTypeDef *out) {
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}
