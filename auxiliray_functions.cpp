#include "auxiliray_functions.h"
#include <math.h>

float map_between(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float* rotate_vector_z(float theta, float* vector)
{
	float res_vector[3] = { 0.0 };

	float R[3][3] = { { cos(theta), -1.0 * sin(theta), 0.0 },
					  { sin(theta), cos(theta),        0.0 },
					  { 0.0,        0.0,               1.0 } };

	for (int i = 0; i < 3; i++)
	{
		float sum = 0.0;
		for (int j = 0; j < 3; j++)
		{
			sum = sum + R[i][j] * vector[j];
		}
		res_vector[i] = sum;
	}

	return res_vector;
}