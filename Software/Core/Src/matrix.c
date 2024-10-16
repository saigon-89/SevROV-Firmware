#include "matrix.h"

#include <math.h>
#include <stdio.h>

void Vector3f_Normalize(Vector3fTypeDef *a) {
  float mag = sqrt(Vector3f_Dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

float Vector3f_Dot(Vector3fTypeDef *a, Vector3fTypeDef *b) {
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void Vector3f_Cross(Vector3fTypeDef *a, Vector3fTypeDef *b,
                  Vector3fTypeDef *out) {
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

float Vector3d_Max(Vector3dTypeDef *a) {
	float max = a->x;

	if (a->y > max) {
		max = a->y;
  }
	if (a->z > max) {
		max = a->z;
  }

  return max;
}

float Vector3d_Min(Vector3dTypeDef *a) {
	float min = a->x;

	if (a->y < min) {
		min = a->y;
  }
	if (a->z < min) {
		min = a->z;
  }

  return min;
}
