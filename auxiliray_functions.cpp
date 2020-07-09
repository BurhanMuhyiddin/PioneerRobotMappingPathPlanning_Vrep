#include "auxiliray_functions.h"
#include "window_parameters.h"
#include "draw_map.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;

extern Nodes *nodes;

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

void multiply_vector(std::vector<float> &matrix, std::vector<float> &vector_, float* res)
{
	// make matrix 4x4. So, add 0 0 0 1 to the end of the matrix vector
	float tmp[] = { 0.0, 0.0, 0.0, 1.0 };
	matrix.insert(matrix.end(), tmp, tmp + 4);

	// insert 1 at the end of vector in order to make it 4x1
	vector_.push_back(1.0);

	// multiply matrix and vector
	for (int i = 0; i < 4; i++)
	{
		float tmpSum = 0.0;
		for (int j = 0; j < 4; j++)
		{
			tmpSum = tmpSum + matrix[j+i*4] * vector_[j];
		}
		res[i] = tmpSum;
	}
}

/*bool is_file_exist(string path)
{
	ifstream fin(fileName.c_str());

	if (fin.fail())
	{
		return false;
	}

	return true;
}*/

void write_map_into_file()
{
	string fileName = "C:\\Users\\39380\\source\\repos\\Pioneer3Robot_MappingAndPthPlanning\\Pioneer3Robot_MappingAndPthPlanning\\NormalMap.txt";

	ofstream normalMap(fileName.c_str());
	if (normalMap.is_open())
	{
		for (int x = 0; x < GRID_COLUMN_NUM; x++)
		{
			for (int y = 0; y < GRID_ROW_NUM; y++)
			{
				normalMap << nodes[x * GRID_ROW_NUM + y].b_obstacle << " ";
			}
			normalMap << "\n";
		}
		normalMap.close();
	}
}

void read_processed_map()
{
	string fileName = "C:\\Users\\39380\\source\\repos\\Pioneer3Robot_MappingAndPthPlanning\\Pioneer3Robot_MappingAndPthPlanning\\ProcessedMap.txt";
	ifstream processedMap(fileName.c_str());
	int val;
	if (processedMap.is_open())
	{
		for (int x = 0; x < GRID_COLUMN_NUM; x++)
		{
			for (int y = 0; y < GRID_ROW_NUM; y++)
			{
				processedMap >> val;
				nodes[x * GRID_ROW_NUM + y].b_obstacle = val;
			}
		}

		processedMap.close();
	}
}