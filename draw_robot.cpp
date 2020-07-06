#include <gl/glut.h>
#include <gl/GLU.h>
#include <gl/GL.h>
#include <iostream>
#include "draw_robot.h"
#include "auxiliray_functions.h"

using namespace std;

/*

In order to rotate the rectangle, I first create a rectangle around center point of (0,0).
Then I apply rotation to this rectangle.
Then I just change the center by adding given x and y.

(There is mixing of [0] [1] indexes. This is because of the coordinate frame of the simulator.)

*/

void draw_robot_itself(float x, float y, float theta)
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

	//Draw rectangle
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_QUADS);

		glVertex2f(x + side1[0], y + side1[1]);
		glVertex2f(x + side2[0], y + side2[1]);
		glVertex2f(x + side3[0], y + side3[1]);
		glVertex2f(x + side4[0], y + side4[1]);

	glEnd();
}
