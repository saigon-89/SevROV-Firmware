#include "matrix.h"

#include <math.h>
#include <stddef.h>

void Vector3f_Normalize(Vector3fTypeDef *a) {
  float mag = sqrt(Vector3f_Dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

float Vector3f_Norm(Vector3fTypeDef *a) {
  return sqrt(a->x * a->x + a->y * a->y + a->z * a->z);
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

void Vector3f_Add(Vector3fTypeDef *a, Vector3fTypeDef *b,
                  Vector3fTypeDef *out) {
  out->x = a->x + b->x;
  out->y = a->y + b->y;
  out->z = a->z + b->z;
}
									
void Vector3f_Scale(Vector3fTypeDef *a, float scale, Vector3fTypeDef *out) {
  out->x = a->x * scale;
	out->y = a->y * scale;
	out->z = a->z * scale;
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

void Quaternion_To_Matrix(QuaternionTypeDef *q, MatrixRotationTypeDef m) {
  float tx  = 2.0f * q->x;
  float ty  = 2.0f * q->y;
  float tz  = 2.0f * q->z;
  float twx = tx * q->w;
  float twy = ty * q->w;
  float twz = tz * q->w;
  float txx = tx * q->x;
  float txy = ty * q->x;
  float txz = tz * q->x;
  float tyy = ty * q->y;
  float tyz = tz * q->y;
  float tzz = tz * q->z;

  m[0][0] = 1.0f - (tyy + tzz);
  m[0][1] = txy - twz;
  m[0][2] = txz + twy;
  m[1][0] = txy + twz;
  m[1][1] = 1.0f - (txx + tzz);
  m[1][2] = tyz - twx;
  m[2][0] = txz - twy;
  m[2][1] = tyz + twx;
  m[2][2] = 1.0f - (txx + tyy);
}

void Quaternion_Multiply(QuaternionTypeDef *q1, QuaternionTypeDef *q2, QuaternionTypeDef *out) {
  out->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  out->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  out->y = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
  out->z = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;
}

void Quaternion_Normalize(QuaternionTypeDef *q) {
  float mag = sqrt(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);

  q->x /= mag;
  q->y /= mag;
  q->z /= mag;
  q->w /= mag;
}

void Quaternion_To_Euler(QuaternionTypeDef *q, Vector3fTypeDef *euler) {
  float qw = q->w;
  float qx = q->x;
  float qy = q->y;
  float qz = q->z;

  float sinr_cosp = 2.0f * (qw * qx + qy * qz);
  float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
  euler->x = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1.0f) {
    euler->y = copysign(3.14f / 2.0f, sinp);
  } else {
    euler->y = asin(sinp);
  }

  float siny_cosp = 2.0f * (qw * qz + qx * qy);
  float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
  euler->z = atan2(siny_cosp, cosy_cosp);
}

void Quaternion_Identity(QuaternionTypeDef *q) {
  q->x = 0.0f;
	q->y = 0.0f;
	q->z = 0.0f;
	q->w = 1.0f;
}

void Matrix3f_Addition(MatrixRotationTypeDef a, MatrixRotationTypeDef b) {
  for (size_t i = 0; i < 3; i++) {
	  for (size_t j = 0; j < 3; j++) {
		  a[i][j] += b[i][j];
		}
	}
}

void Matrix3f_Multiply(MatrixRotationTypeDef a, MatrixRotationTypeDef b, MatrixRotationTypeDef out) {
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      out[i][j] = 0.0f;
      for (size_t k = 0; k < 3; k++) {
        out[i][j] += a[i][k] * b[k][j];
      } 
    }
  }
}
