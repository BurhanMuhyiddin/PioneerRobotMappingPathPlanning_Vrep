#include <iostream>
#include <vector>
#include <gl/glut.h>
#include <gl/GLU.h>
#include <gl/GL.h>
#include "b0RemoteApi.h"
#include "window_parameters.h"
#include "draw_robot.h"
#include "auxiliray_functions.h"

using namespace std;

// Definitions of callback functions
void display_callback();
void reshape_callback(int, int);
void keyboard_callback(unsigned char, int, int);
void mouse_callback(int, int, int, int);
void timer_callback(int);
// End of definitions of callback functions

// Definition of user defined variables
b0RemoteApi* cl = NULL;
int mainWindow = 0;
int pioneerRobotHandle = 0;
vector<float> robotPos; // robot's position
vector<float> robotOrn; // robot's orientation
// End of definition of user defined variables

void init()
{
	glClearColor(0.0, 0.0, 0.0, 1.0);
}

int main(int argc, char** argv)
{
	// b0 remote api initialization
	b0RemoteApi client("b0RemoteApi_c++Client", "b0RemoteApiAddOn");
	cl = &client;

	// get some object handles
	pioneerRobotHandle = b0RemoteApi::readInt(cl->simxGetObjectHandle("Pioneer_p3dx", cl->simxServiceCall()), 1);

	// opengl initialization
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowPosition(200, 100);
	glutInitWindowSize(W_W, W_H);
	mainWindow = glutCreateWindow("Pioneer3Control");

	glutDisplayFunc(display_callback);
	glutReshapeFunc(reshape_callback);
	glutKeyboardFunc(keyboard_callback);
	glutMouseFunc(mouse_callback);
	glutTimerFunc(0, timer_callback, 0);

	init();

	glutMainLoop();
}

int counter = 0;

void display_callback()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();

	try
	{
		b0RemoteApi::readFloatArray(cl->simxGetObjectPosition(pioneerRobotHandle, -1, cl->simxServiceCall()), robotPos, 1);
		b0RemoteApi::readFloatArray(cl->simxGetObjectOrientation(pioneerRobotHandle, -1, cl->simxServiceCall()), robotOrn, 1);
		draw_robot_itself(map_between(robotPos[1], 2.210, -2.210, 1, (GRID_COLUMN_NUM - 2) * 1.0),
			map_between(robotPos[0], 2.210, -2.210, 1, (GRID_ROW_NUM - 2) * 1.0),
			robotOrn[2]);
	}
	catch (const std::exception& exc)
	{
		glutDestroyWindow(mainWindow);
	};

	glutSwapBuffers();
}

void reshape_callback(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, (GRID_COLUMN_NUM-1.0), (GRID_ROW_NUM-1.0), 0.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
}

void keyboard_callback(unsigned char key, int x, int y)
{
	switch (key)
	{
	}
}

void mouse_callback(int button, int state, int mouseX, int mouseY)
{
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{

	}
}

void timer_callback(int)
{
	glutPostRedisplay();
	glutTimerFunc(1000 / FPS, timer_callback, 0);
}