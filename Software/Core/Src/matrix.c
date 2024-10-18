#include "matrix.h"

#include <math.h>
#include <stdio.h>

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

#if 0
{
  // NOTE if inlined, then gcc 4.2 and 4.4 get rid of the temporary (not gcc 4.3 !!)
  // if not inlined then the cost of the return by value is huge ~ +35%,
  // however, not inlining this function is an order of magnitude slower, so
  // it has to be inlined, and so the return by value is not an issue
  Matrix3 res;

  const Scalar tx  = Scalar(2)*q->x;
  const Scalar ty  = Scalar(2)*q->y;
  const Scalar tz  = Scalar(2)*q->z;
  const Scalar twx = tx*q->w;
  const Scalar twy = ty*q->w;
  const Scalar twz = tz*q->w;
  const Scalar txx = tx*q->x;
  const Scalar txy = ty*q->x;
  const Scalar txz = tz*q->x;
  const Scalar tyy = ty*q->y;
  const Scalar tyz = tz*q->y;
  const Scalar tzz = tz*q->z;

  res.coeffRef(0,0) = Scalar(1)-(tyy+tzz);
  res.coeffRef(0,1) = txy-twz;
  res.coeffRef(0,2) = txz+twy;
  res.coeffRef(1,0) = txy+twz;
  res.coeffRef(1,1) = Scalar(1)-(txx+tzz);
  res.coeffRef(1,2) = tyz-twx;
  res.coeffRef(2,0) = txz-twy;
  res.coeffRef(2,1) = tyz+twx;
  res.coeffRef(2,2) = Scalar(1)-(txx+tyy);

  return res;
}
#endif

void Quaternion_To_Matrix(QuaternionTypeDef *q, MatrixRotationTypeDef m) {
#if 0
  float xx = q->x * q->x;
  float xy = q->x * q->y;
  float xz = q->x * q->z;
  float xw = q->x * q->w;

  float yy = q->y * q->y;
  float yz = q->y * q->z;
  float yw = q->y * q->w;

  float zz = q->z * q->z;
  float zw = q->z * q->w;
    
  m[0][0] = 1.0f - 2.0f * (yy + zz);
  m[0][1] = 2.0f * (xy - zw);
  m[0][2] = 2.0f * (xz + yw);

  m[1][0] = 2.0f * (xy + zw);
  m[1][1] = 1.0f - 2.0f * (xx + zz);
  m[1][2] = 2.0f * (yz - xw);

  m[2][0] = 2.0f * (xz - yw);
  m[2][1] = 2.0f * (yz + xw);
  m[2][2] = 1.0f - 2.0f * (xx + yy);
#else
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
#endif
}

#if 0
    (
      a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
      a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
      a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z(),
      a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x()
    );
#endif

void Quaternion_Multiply(QuaternionTypeDef *q1, QuaternionTypeDef *q2, QuaternionTypeDef *out) {
  out->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  out->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  out->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q1->x;
	out->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q1->w;
}

void Quaternion_Normalize(QuaternionTypeDef *q) {
  float mag = sqrt(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);

  q->x /= mag;
  q->y /= mag;
  q->z /= mag;
  q->w /= mag;
}

void Quaternion_To_Euler(QuaternionTypeDef *q, Vector3fTypeDef *euler) {
    // Extract the values from Quaternion
    float qw = q->w;
    float qx = q->x;
    float qy = q->y;
    float qz = q->z;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    euler->x = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0f) {
        euler->y = copysign(3.14f / 2.0f, sinp); // Use 90 degrees if out of range
    } else {
        euler->y = asin(sinp);
    }

    // Yaw (z-axis rotation)
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
