#ifndef __ROSBOT_KINEMATICS_H__
#define __ROSBOT_KINEMATICS_H__

#include <RosbotDrive.h>

#define ROBOT_WIDTH 0.215         // 0.22 0.195
#define DIAMETER_MODIFICATOR 1.106 // 1.24, 1.09, 1.164
#define TYRE_DEFLATION 1.042      // theoretical distance / real distance
#define GEAR_RATIO 34.014
// #define GEAR_RATIO 20.4        // red wheels
#define ENCODER_CPR 48
#define ROBOT_WIDTH_HALF ROBOT_WIDTH/2.0
#define WHEEL_DIAMETER 0.085
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0
#define POLARITY 0b00111100
// #define POLARITY 0b11000011 // red wheels
#define MOTOR_FR MOTOR1
#define MOTOR_FL MOTOR4
#define MOTOR_RR MOTOR2
#define MOTOR_RL MOTOR3

namespace rosbot_kinematics {

extern RosbotWheel custom_wheel_params;

struct Odometry
{
    float wheel_FR_ang_pos;  // radians
    float wheel_FL_ang_pos;  // radians
    float wheel_RR_ang_pos;  // radians
    float wheel_RL_ang_pos;  // radians
    float wheel_L_ang_pos;   // radians
    float wheel_R_ang_pos;   // radians
    float wheel_L_ang_vel;   // radians per second
    float wheel_R_ang_vel;   // radians per second
    float robot_angular_pos; // radians
    float robot_angular_vel; // radians per second
    float robot_x_pos;       // meters
    float robot_y_pos;       // meters
    float robot_x_vel;       // meters per second
    float robot_y_vel;       // meters per second
};

union RosbotOdometry
{
    Odometry odom;
    float buffor[14];
};

void setRosbotSpeed(RosbotDrive & drive, float linear, float angular);
void updateRosbotOdometry(RosbotDrive & drive, RosbotOdometry & odom, float dtime);
void resetRosbotOdometry(RosbotDrive & drive, RosbotOdometry & odom);

}
#endif /* __ROSBOT_KINEMATICS_H__ */