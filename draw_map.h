#ifndef DRAW_MAP_H_INCLUDED
#define DRAW_MAP_H_INCLUDED

struct Nodes
{
	int x;
	int y;
	int cost;
	int h; // heuristic cost
	int g; // g cost
	int f; // f cost
	bool b_obstacle;
	bool b_visited;
	int parent;
};

void init_map();
void draw_map();

#endif // !DRAW_MAP_H_INCLUDED
