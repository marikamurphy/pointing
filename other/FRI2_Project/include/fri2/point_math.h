#ifndef _POINT_MATH_H_
#define _POINT_MATH_H_

#include <ros/ros.h>
#include <cmath>
#include <Eigen/Eigen>
extern float dotProduct(const float *v1, const float *v2, int N);
extern void crossProduct(const float v1[3], const float v2[3], float result[3]);
extern float pointLineDistance3d(const float v1[3], const float v2[3], const float pt[3]);
extern float det(const float *matrix, int n);
extern float magnitude(const float *vec, int n);

#endif
