#include "drive_robot.h"
#include "robot_parameters.h"
#include "b0RemoteApi.h"

extern b0RemoteApi* cl;
extern int rightMotorHandle;
extern int leftMotorHandle;

void calculate_robot_kinematics(float velocity, float steer_angle, float* buf)
{
	float rightWheelVelocity = velocity + (DISTANCE_BETWEEN_WHEELS)* steer_angle;
	float leftWheelVelocity = velocity - (DISTANCE_BETWEEN_WHEELS)* steer_angle;
	float rightAngularVelocity = rightWheelVelocity / WHEEL_RADIUS;
	float leftAngularVelocity = leftWheelVelocity / WHEEL_RADIUS;
	buf[0] = rightAngularVelocity;
	buf[1] = leftAngularVelocity;
}

void stop_robot()
{
	cl->simxSetJointTargetVelocity(rightMotorHandle, 0.0, cl->simxServiceCall());
	cl->simxSetJointTargetVelocity(leftMotorHandle, 0.0, cl->simxServiceCall());
}