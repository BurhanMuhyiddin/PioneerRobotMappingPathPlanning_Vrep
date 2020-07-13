#include "draw_map.h"
#include <gl/glut.h>
#include <gl/GLU.h>
#include <gl/GL.h>
#include "window_parameters.h"

Nodes *nodes = nullptr;

void init_map()
{
	nodes = new Nodes[GRID_ROW_NUM * GRID_ROW_NUM];

	for (int x = 0; x < GRID_COLUMN_NUM; x++)
	{
		for (int y = 0; y < GRID_ROW_NUM; y++)
		{
			nodes[x * GRID_ROW_NUM + y].x = x;
			nodes[x * GRID_ROW_NUM + y].y = y;
			nodes[x * GRID_ROW_NUM + y].b_obstacle = false;
			nodes[x * GRID_ROW_NUM + y].b_visited = false;
		}
	}
}

void draw_map()
{
	for (int x = 0; x < GRID_COLUMN_NUM; x++)
	{
		for (int y = 0; y < GRID_ROW_NUM; y++)
		{
			if (!nodes[x * GRID_ROW_NUM + y].b_obstacle) // if empty
			{
				// draw grey rectangle
				glColor3f(0.4, 0.4, 0.4);
			}
			else // if not empty
			{
				// draw black rectangle
				glColor3f(0.0, 0.0, 0.0);
			}
			glBegin(GL_QUADS);
				glVertex2f(x*1.0-0.5, y*1.0+0.5);
				glVertex2f(x*1.0+0.5, y*1.0+0.5);
				glVertex2f(x*1.0+0.5, y*1.0-0.5);
				glVertex2f(x*1.0-0.5, y*1.0-0.5);
			glEnd();
		}
	}
}