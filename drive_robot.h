#ifndef DRIVE_ROBOT_H_INCLUDED
#define DRIVE_ROBOT_H_INCLUDED

void calculate_robot_kinematics(float velocity, float steer_angle, float* buf);
void stop_robot();

#endif // !DRIVE_ROBOT_H_INCLUDED

