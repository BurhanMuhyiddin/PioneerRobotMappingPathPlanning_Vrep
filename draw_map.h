#ifndef DRAW_MAP_H_INCLUDED
#define DRAW_MAP_H_INCLUDED

struct Nodes
{
	int x;
	int y;
	bool b_obstacle;
	bool b_observed;
	int parent;
};

void init_map();
void draw_map();

#endif // !DRAW_MAP_H_INCLUDED
