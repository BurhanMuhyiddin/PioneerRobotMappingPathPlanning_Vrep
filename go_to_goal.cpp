#include "go_to_goal.h"
#include "b0RemoteApi.h"
#include "drive_robot.h"
#include "robot_parameters.h"
#include "auxiliray_functions.h"
#include "window_parameters.h"
#include <vector>

using namespace std;

vector<float> goalPos;

extern b0RemoteApi* cl;
extern int pioneerRobotHandle;
extern int rightMotorHandle;
extern int leftMotorHandle;
extern int goalDummyHandle;
extern int midPointDummyHandle;

void robot_go_to_goal(int goalX, int goalY)
{
	float midX = map_between(goalX, 1, (GRID_COLUMN_NUM - 1) * 1.0, 2.47, -2.47);
	float midY = map_between(goalY, 1, (GRID_ROW_NUM - 1) * 1.0, 2.47, -2.47);
	float totPos[] = { midX, midY, 0.0 };

	cl->simxSetObjectPosition(midPointDummyHandle, -1, totPos, cl->simxServiceCall());
	b0RemoteApi::readFloatArray(cl->simxGetObjectPosition(midPointDummyHandle, pioneerRobotHandle, cl->simxServiceCall()), goalPos, 1);

	float dis = sqrt(pow(goalPos[0], 2) + pow(goalPos[1], 2));
	float phi = atan2(goalPos[1], goalPos[0]);

	//if (dis < 0.1)
	//{
	//	stop_robot();
	//}
	//else 
	//{
		float angular_velocities[2] = { 0.0 };
		calculate_robot_kinematics(ROBOT_VELOCITY, 0.8*phi, angular_velocities);
		cl->simxSetJointTargetVelocity(rightMotorHandle, angular_velocities[0], cl->simxServiceCall());
		cl->simxSetJointTargetVelocity(leftMotorHandle, angular_velocities[1], cl->simxServiceCall());
	//}
}