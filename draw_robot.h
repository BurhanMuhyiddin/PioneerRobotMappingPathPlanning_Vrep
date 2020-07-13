#ifndef DRAW_ROBOT_H_INCLUDED
#define DRAW_ROBOT_H_INCLUDED

#define R_W				4.0 // Robot width
#define R_H				6.0 // Robot height
#define SENSOR_RANGE	10.0
#define RADIUS			3.0

void draw_just_robot(float x, float y, float theta);
void draw_robot_and_additional_parts(float x, float y, float theta);
void visualize_range(int sX, int sY, int gX, int gY);
void limit_range(float* pX, float* pY);

#endif // !DRAW_ROBOT_H_INCLUDED