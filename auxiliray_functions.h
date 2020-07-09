#ifndef AUXILIRAY_FUNCTIONS_H_INCLUDED
#define AUXILIRAY_FUNCTIONS_H_INCLUDED

#include <vector>
#include <string>

float map_between(float x, float in_min, float in_max, float out_min, float out_max);
float* rotate_vector_z(float theta, float* vector);
void multiply_vector(std::vector<float> &matrix, std::vector<float> &vector_, float* res);
void write_map_into_file();
void read_processed_map();

#endif // !AUXILIRAY_FUNCTIONS_H_INCLUDED
