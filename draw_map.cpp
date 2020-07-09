#include "draw_map.h"
#include <gl/glut.h>
#include <gl/GLU.h>
#include <gl/GL.h>
#include "window_parameters.h"

Nodes *nodes = nullptr;

void init_map()
{
	nodes = new Nodes[GRID_COLUMN_NUM * GRID_ROW_NUM];

	for (int x = 0; x < GRID_COLUMN_NUM; x++)
	{
		for (int y = 0; y < GRID_ROW_NUM; y++)
		{
			nodes[x * GRID_ROW_NUM + y].x = x;
			nodes[x * GRID_ROW_NUM + y].y = y;
			nodes[x * GRID_ROW_NUM + y].b_obstacle = false;
			nodes[x * GRID_ROW_NUM + y].b_observed = false;
		}
	}
}

void draw_map()
{
	for (int x = 0; x < GRID_COLUMN_NUM; x++)
	{
		for (int y = 0; y < GRID_ROW_NUM; y++)
		{
			//if (!nodes[x * GRID_ROW_NUM + y].b_observed) // if not observed
			//{
				// draw gray rectangles
			//	glColor3f(0.4, 0.4, 0.4);
			//}
			if (!nodes[x * GRID_ROW_NUM + y].b_obstacle) // if empty
			{
				// draw white rectangle
				glColor3f(1.0, 1.0, 1.0);
			}
			else // if not empty
			{
				// draw black rectangle
				glColor3f(0.0, 0.0, 0.0);
			}
			//glRectd(x, y, x + 1, y + 1);
			glBegin(GL_QUADS);
				glVertex2f(x*1.0-0.5, y*1.0+0.5);
				glVertex2f(x*1.0+0.5, y*1.0+0.5);
				glVertex2f(x*1.0+0.5, y*1.0-0.5);
				glVertex2f(x*1.0-0.5, y*1.0-0.5);
			glEnd();
		}
	}
}