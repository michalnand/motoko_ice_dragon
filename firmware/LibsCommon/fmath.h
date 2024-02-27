#ifndef _MATH_H_
#define _MATH_H_

#include "math_t.h"

#define PI              ((float)3.14159265358979323846)
#define SQRT2           ((float)1.4142135623730951)
#define EPSILON         ((float)0.00000001)
#define RAD_TO_DEG      ((float)180.0/PI)
#define DEG_TO_RAD      ((float)PI/180.0)

template<class DType>
struct Vect2d
{   
    DType x, y;
};

template<class DType>
struct Vect3d
{   
    DType x, y, z;
};

template<class DType>
struct Vect4d
{   
    DType x, y, z, w;
};


float clip(float value, float min, float max);

unsigned int rand();

float fsqrt(float x);

float fsin(float x);
float fcos(float x);
float ftan(float x);
float fsec(float x);

float fasin(float x);
float facos(float x);
float fatan(float x);


float fatan2(float x, float y);

float exp(float x);
float sigmoid(float x);

//compute difference from two consecutive values
float first_difference_1(float x_now, float *x, float dt);

//compute difference from four consecutive values
float first_difference_4(float x_now, float *x, float dt);

//use simpson rule for integration
//float *x holds 3 last values
float integrate_step(float x_now, float *x);

#endif