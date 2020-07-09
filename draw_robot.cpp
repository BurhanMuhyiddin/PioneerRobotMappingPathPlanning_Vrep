#include <gl/glut.h>
#include <gl/GLU.h>
#include <gl/GL.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "draw_robot.h"
#include "b0RemoteApi.h"
#include "auxiliray_functions.h"
#include "window_parameters.h"
#include "draw_map.h"
#include <Windows.h>

using namespace std;

vector<float> sensor5Pos; // ultrasonic sensr fifth's position
vector<float> detPoint;

int ultrasonicSensor5Handle = 0;
extern Nodes *nodes;

extern b0RemoteApi* cl;
extern int pioneerRobotHandle;

/*

In order to rotate the rectangle, I first create a rectangle around center point of (0,0).
Then I apply rotation to this rectangle.
Then I just change the center by adding given x and y.

(There is mixing of [0] [1] indexes. This is because of the coordinate frame of the simulator.)

*/

void draw_just_robot(float x, float y, float theta)
{
	theta = theta * (-1.0);

	// sides of rectangle
	float side1[3] = { -1.0 * R_W / 2.0, R_H / 2.0, 0.0 };
	float side2[3] = { R_W / 2.0, R_H / 2.0, 0.0 };
	float side3[3] = { R_W / 2.0, -1.0 * R_H / 2.0, 0.0 };
	float side4[3] = { -1.0 * R_W / 2.0, -1.0 * R_H / 2.0, 0.0 };

	// Apply rotation to each point
	float* res = rotate_vector_z(theta, side1);
	side1[0] = res[0]; side1[1] = res[1];

	res = rotate_vector_z(theta, side2);
	side2[0] = res[0]; side2[1] = res[1];

	res = rotate_vector_z(theta, side3);
	side3[0] = res[0]; side3[1] = res[1];

	res = rotate_vector_z(theta, side4);
	side4[0] = res[0]; side4[1] = res[1];

	// Draw rectangle
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_QUADS);

		glVertex2f(x + side1[0], y + side1[1]);
		glVertex2f(x + side2[0], y + side2[1]);
		glVertex2f(x + side3[0], y + side3[1]);
		glVertex2f(x + side4[0], y + side4[1]);

	glEnd();
}

void draw_robot_and_additional_parts(float x, float y, float theta)
{
	draw_just_robot(x, y, theta);

	// Draw ultrasonic sensor line
	vector<float> trMatrix; // transformation matrix 
	float detAbsPoint[4]; // absolute coordinates of the detected point
	int detectionState = 0;

	ultrasonicSensor5Handle = b0RemoteApi::readInt(cl->simxGetObjectHandle("Pioneer_p3dx_ultrasonicSensor5", cl->simxServiceCall()), 1);

	b0RemoteApi::readFloatArray(cl->simxGetObjectPosition(ultrasonicSensor5Handle, -1, cl->simxServiceCall()), sensor5Pos, 1); // sensor position relative to the world
	std::vector<msgpack::object>* proximitySensorRetValue = cl->simxReadProximitySensor(ultrasonicSensor5Handle, cl->simxServiceCall());
	Sleep(5);
	proximitySensorRetValue = cl->simxReadProximitySensor(ultrasonicSensor5Handle, cl->simxServiceCall());

	if (b0RemoteApi::readInt(proximitySensorRetValue, 1) == 1)
	{
		b0RemoteApi::readFloatArray(proximitySensorRetValue, detPoint, 3); // detected points relative to the sensor
		b0RemoteApi::readFloatArray(cl->simxGetObjectMatrix(ultrasonicSensor5Handle, -1, cl->simxServiceCall()), trMatrix, 1); // transformation matrix from sensor to the world frame
		multiply_vector(trMatrix, detPoint, detAbsPoint); // get absolute coordinates of the detected point
		//cout << detAbsPoint[0] << " " << detAbsPoint[1] << endl;
		// draw line between detected point and the sensor
		float x1 = map_between(sensor5Pos[1], 2.47, -2.47, 0, (GRID_ROW_NUM - 1) * 1.0);
		float y1 = map_between(sensor5Pos[0], 2.47, -2.47, 0, (GRID_COLUMN_NUM - 1) * 1.0);
		float x2 = map_between(detAbsPoint[1], 2.47, -2.47, 0, (GRID_ROW_NUM - 1) * 1.0);
		float y2 = map_between(detAbsPoint[0], 2.47, -2.47, 0, (GRID_COLUMN_NUM - 1) * 1.0);
		//cout << sensor5Pos[1] << "  " << sensor5Pos[0] << endl;
		
		limit_range(&x2, &y2);
		nodes[(int)round(x2) * GRID_ROW_NUM + (int)round(y2)].b_obstacle = true;
		//visualize_range(x1, y1, x2, y2);

		glColor3f(1.0, 1.0, 0.1);
		glBegin(GL_LINES);
			glVertex2f(x1, y1);
			glVertex2f(x2, y2);
		glEnd();
	}
	else
	{
		// get z angle of the sensor with respect to the world
		vector<float> sensor5Or; // sensor orientation
		b0RemoteApi::readFloatArray(cl->simxGetObjectOrientation(pioneerRobotHandle, -1, cl->simxServiceCall()), sensor5Or, 1);
		float z_angle = sensor5Or[2];

		// Draw line
		float x1 = map_between(sensor5Pos[1], 2.47, -2.47, 0, (GRID_ROW_NUM - 1) * 1.0);
		float y1 = map_between(sensor5Pos[0], 2.47, -2.47, 0, (GRID_COLUMN_NUM - 1) * 1.0);
		float x2 = x1 - SENSOR_RANGE * sin(z_angle);
		float y2 = y1 - SENSOR_RANGE * cos(z_angle);

		limit_range(&x2, &y2);
		//visualize_range(x1, y1, x2, y2);

		glColor3f(1.0, 1.0, 0.1);
		glBegin(GL_LINES);
			glVertex2f(x1, y1);
			glVertex2f(x2, y2);
		glEnd();
	}
}

void visualize_range(int sX, int sY, int gX, int gY)
{
	int dx = gX - sX, dy = gY - sY;
	int nx = abs(dx), ny = abs(dy);
	int sign_x = dx > 0 ? 1 : -1, sign_y = dy > 0 ? 1 : -1;

	for (int ix = 0, iy = 0; ix < nx || iy < ny;) {
		if ((1 + 2 * ix) * ny == (1 + 2 * iy) * nx) {
			// next step is diagonal
			sX += sign_x;
			sY += sign_y;
			ix++;
			iy++;
		}
		else if ((1 + 2 * ix) * ny < (1 + 2 * iy) * nx) {
			// next step is horizontal
			sX += sign_x;
			ix++;
		}
		else {
			// next step is vertical
			sY += sign_y;
			iy++;
		}
		nodes[sX * GRID_ROW_NUM + sY].b_observed = true;
		//if (nodes[sX * Y_MAX + sY].bObstacle || isAroundObstacle(sX, sY))	return false;
	}
}

void limit_range(float* pX, float* pY)
{

	if (*pX > (GRID_COLUMN_NUM - 1)*1.0)	*pX = (GRID_COLUMN_NUM - 1)*1.0;
	else if (*pX < 0)						*pX = 0;

	if (*pY > (GRID_ROW_NUM - 1)*1.0)		*pY = (GRID_ROW_NUM - 1)*1.0;
	else if (*pY < 0)						*pY = 0;
}