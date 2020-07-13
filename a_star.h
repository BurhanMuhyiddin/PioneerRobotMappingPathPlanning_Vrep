#ifndef A_STAR_H_INCLUDED
#define A_STAR_H_INCLUDED

#define INF				9999999

void solve_a_star();
void get_initial_and_goal_positions(float robotX, float robotY);
void init_a_star(float robotX, float robotY);
int calculate_heuristic(int x, int y);
void calculate_min_f_cost(int* val);
void update_costs(int current, int index);
bool check_node(int index);
bool check_node_environment(int index);
void get_mid_points();

#endif // !A_STAR_H_INCLUDED
