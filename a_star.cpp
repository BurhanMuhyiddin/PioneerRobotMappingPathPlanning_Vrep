#include "a_star.h"
#include "window_parameters.h"
#include "draw_map.h"
#include "b0RemoteApi.h"
#include <vector>
#include "auxiliray_functions.h"
#include "draw_robot.h"
#include "go_to_goal.h"
#include <algorithm>

using namespace std;

int init_point, goal_point = 0;
int final_point = 0;

extern Nodes *nodes;
extern b0RemoteApi* cl;
extern vector<int>midPoints;
extern int goalDummyHandle;

void init_a_star(float robotX, float robotY)
{
	get_initial_and_goal_positions(robotX, robotY);

	//robot_go_to_goal(nodes[goal_point].x, nodes[goal_point].y);

	// initialize cost of each node
	for (int x = 0; x < GRID_COLUMN_NUM; x++)
	{
		for (int y = 0; y < GRID_ROW_NUM; y++)
		{
			nodes[x*GRID_ROW_NUM + y].g = INF;
			nodes[x*GRID_ROW_NUM + y].f = INF;
			nodes[x*GRID_ROW_NUM + y].h = calculate_heuristic(x, y);
			nodes[x*GRID_ROW_NUM + y].cost = abs(nodes[init_point].x - x) + abs(nodes[init_point].y - y);
		}
	}

	// g cost of start node has to be zero, so, f cost becomes eqaul to h cost. Because, f = g+h
	nodes[init_point].g = 0;
	nodes[init_point].f = calculate_heuristic(nodes[init_point].x, nodes[init_point].y);
	//cout << "I passed here" << endl;
}

void get_initial_and_goal_positions(float robotX, float robotY)
{
	int res = 0;
	vector<float> dummyPos;

	init_point = (int)(round(robotX) * (GRID_ROW_NUM * 1.0) + round(robotY)); // get robot's current position as start point

	// get dummy's position as goal point
	b0RemoteApi::readFloatArray(cl->simxGetObjectPosition(goalDummyHandle, -1, cl->simxServiceCall()), dummyPos, 1);
	float tmpX = map_between(dummyPos[1], 2.47, -2.47, 1, (GRID_COLUMN_NUM - 1) * 1.0);
	float tmpY = map_between(dummyPos[0], 2.47, -2.47, 1, (GRID_COLUMN_NUM - 1) * 1.0);
	goal_point = (int)(round(tmpX) * (GRID_ROW_NUM * 1.0) + round(tmpY));
}

int calculate_heuristic(int x, int y)
{
	int gX = nodes[goal_point].x;
	int gY = nodes[goal_point].y;

	return (abs(gX - x) + abs(gY - y));
}

void calculate_min_f_cost(int* val) // returns minimum f cost
{
	int minFCost = INF;
	int index = 0;
	for (int x = 0; x < GRID_COLUMN_NUM; x++)
	{
		for (int y = 0; y < GRID_ROW_NUM; y++)
		{
			int tmp = nodes[x*GRID_ROW_NUM + y].f;
			if (tmp < minFCost)
			{
				minFCost = tmp;
				index = x * GRID_ROW_NUM + y;
			}
		}
	}

	val[0] = minFCost;
	val[1] = index;
}

bool check_node(int index)
{
	if (nodes[index].b_visited == false && nodes[index].b_obstacle == false && check_node_environment(index))
		return true;

	return false;
}

bool check_node_environment(int index) // this is to check if there is obstacle near robot radius
{
	int tX = nodes[index].x, tY = nodes[index].y;
	for (int r = 1; r <= RADIUS; r++)
	{
		if (tY - 1 > 0)
		{
			for (int x = tX - r; x <= tX + r; x++)
			{
				if (x > 0 && x < GRID_COLUMN_NUM)
				{
					if (nodes[x*GRID_ROW_NUM + tY - 1].b_obstacle)	return false;
				}
			}
		}
		if (tY + 1 < GRID_ROW_NUM)
		{
			for (int x = tX - r; x <= tX + r; x++)
			{
				if (x > 0 && x < GRID_COLUMN_NUM)
				{
					if (nodes[x*GRID_ROW_NUM + tY + 1].b_obstacle)	return false;
				}
			}
		}
		if (tX - 1 > 0)
		{
			for (int y = tY - r; y <= tY + r; y++)
			{
				if (y > 0 && y < GRID_ROW_NUM)
				{
					if (nodes[(tX - 1)*GRID_ROW_NUM + y].b_obstacle)	return false;
				}
			}
		}
		if (tX + 1 < GRID_COLUMN_NUM)
		{
			for (int y = tY - r; y <= tY + r; y++)
			{
				if (y > 0 && y < GRID_ROW_NUM)
				{
					if (nodes[(tX + 1)*GRID_ROW_NUM + y].b_obstacle)	return false;
				}
			}
		}
	}

	return true;
}

void update_costs(int current, int index)
{
	if (check_node(index))
	{
		int t = nodes[current].g + abs(nodes[current].cost - nodes[index].cost);
		if (t < nodes[index].g)
		{
			nodes[index].g = t;
			nodes[index].f = nodes[index].g + nodes[index].h;
			nodes[index].parent = current;
		}
	}
}

void solve_a_star()
{
	int current_node[2] = { 0 };
	while (true)
	{
		// get minimum f value
		calculate_min_f_cost(current_node);
		int minfValue = current_node[0];
		int current = current_node[1];

		// check whether you reached the goal
		if (current == goal_point || minfValue == INF)
		{
			final_point = current;
			break;
		}

		// indicate current node as visited
		nodes[current].b_visited = true;

		// visit each neighbour
		// get x and y coordinates
		// neighbours: (x-1, y) (x+1, y) (x, y-1) (x, y+1)
		int x_ = nodes[current].x;
		int y_ = nodes[current].y;

		if (x_ > 0)									update_costs(current, (x_ - 1) * GRID_ROW_NUM + y_);
		if (x_ < (GRID_COLUMN_NUM - 1))				update_costs(current, (x_ + 1) * GRID_ROW_NUM + y_);
		if (y_ > 0)									update_costs(current, x_ * GRID_ROW_NUM + y_ - 1);
		if (y_ < (GRID_ROW_NUM - 1))				update_costs(current, x_ * GRID_ROW_NUM + y_ + 1);

		nodes[current].f = INF;
	}
}

void get_mid_points()
{
	int node1 = final_point;
	int node2 = nodes[final_point].parent;

	while (node2 != init_point)
	{
		//cout << nodes[node1].x << " " << nodes[node1].y << endl;
		midPoints.push_back(node1);

		node1 = node2;
		node2 = nodes[node2].parent;
	}

	std::reverse(midPoints.begin(), midPoints.end());
}