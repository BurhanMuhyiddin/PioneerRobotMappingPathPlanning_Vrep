#include <iostream>
#include <vector>
#include <gl/glut.h>
#include <gl/GLU.h>
#include <gl/GL.h>
#include "b0RemoteApi.h"
#include "window_parameters.h"
#include "draw_robot.h"
#include "auxiliray_functions.h"
#include "draw_map.h"
#include "menu_parameters.h"

using namespace std;

// Definitions of callback functions
void display_callback();
void reshape_callback(int, int);
void menu_callback(int);
void timer_callback(int);
// End of definitions of callback functions

void initMenu();

// Definition of user defined variables
b0RemoteApi* cl = NULL;
int mainWindow = 0;
int sub1, sub2;
bool flag1 = false;
bool map_mode = false, path_planning_mode = false;
int pioneerRobotHandle = 0;
vector<float> robotPos; // robot's position
vector<float> robotOrn; // robot's orientation
vector<float> testOrn;
// End of definition of user defined variables

void init()
{
	glClearColor(0.0, 0.0, 0.0, 1.0);
	initMenu();
	init_map();
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
	glutTimerFunc(0, timer_callback, 0);

	init();

	glutMainLoop();
}

void display_callback()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();

	try
	{
		b0RemoteApi::readFloatArray(cl->simxGetObjectPosition(pioneerRobotHandle, -1, cl->simxServiceCall()), robotPos, 1);
		b0RemoteApi::readFloatArray(cl->simxGetObjectOrientation(pioneerRobotHandle, -1, cl->simxServiceCall()), robotOrn, 1);
		//cout << robotPos[0] << " " << robotPos[1] << endl;

		draw_map();

		if (map_mode)
		{
			draw_robot_and_additional_parts(map_between(robotPos[1], 2.47, -2.47, 1, (GRID_COLUMN_NUM - 1) * 1.0),
				map_between(robotPos[0], 2.47, -2.47, 1, (GRID_ROW_NUM - 1) * 1.0),
				robotOrn[2]);
			if (b0RemoteApi::readInt(cl->simxGetIntSignal("s_finish", cl->simxServiceCall()), 1) == 1)
			{
				// write map into the file in order to be further prosessed
				cout << "Write map into file..." << endl;
				write_map_into_file();
				cout << "Finished writing..." << endl;
				map_mode = false;
			}
		}
		else if (path_planning_mode)
		{
			if (!flag1)
			{
				cout << "Read processed map..." << endl;
				read_processed_map();
				cout << "Finished reading..." << endl;
				flag1 = true;
			}
			draw_just_robot(map_between(robotPos[1], 2.210, -2.210, 1, (GRID_COLUMN_NUM - 2) * 1.0),
				map_between(robotPos[0], 2.210, -2.210, 1, (GRID_ROW_NUM - 2) * 1.0),
				robotOrn[2]);
			// do path planning here
		}
	}
	catch (const std::exception& exc)
	{
	//	cerr << exc.what() << endl;
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

void initMenu()
{
	sub1 = glutCreateMenu(menu_callback);
	glutAddMenuEntry("map mode", MAP_CREATION);
	glutAddMenuEntry("path planning mode", PATH_PLANNING);

	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void menu_callback(int item)
{
	switch (item)
	{
	case MAP_CREATION:
		map_mode = true;
		path_planning_mode = false;
		break;
	case PATH_PLANNING:
		path_planning_mode = true;
		map_mode = false;
		break;
	}
}

void timer_callback(int)
{
	glutPostRedisplay();
	glutTimerFunc(1000 / FPS, timer_callback, 0);
}