#ifndef __ROSBOT_KINEMATICS_H__
#define __ROSBOT_KINEMATICS_H__

#include <RosbotDrive.h>

#if defined(ROSBOT_KINEMATICS_TEST)

#define ROBOT_WIDTH 0.19
#define DIAMETER_MODIFICATOR 1.0f
#define TYRE_DEFLATION 1.0f
#define GEAR_RATIO 34.014
#define ENCODER_CPR 48
#define ROBOT_WIDTH_HALF ROBOT_WIDTH/2.0f
#define WHEEL_DIAMETER 0.063
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0f
#define POLARITY 0b00010011
#define MOTOR_FR MOTOR1
#define MOTOR_FL MOTOR2
#define MOTOR_RR MOTOR3
#define MOTOR_RL MOTOR4

#else

#define ROBOT_WIDTH 0.215         // 0.22 0.195
#define DIAMETER_MODIFICATOR 1.164 // 1.24, 1.09
#define TYRE_DEFLATION 1.042      // theoretical distance / real distance
#define GEAR_RATIO 34.014
#define ENCODER_CPR 48
#define ROBOT_WIDTH_HALF ROBOT_WIDTH/2.0
#define WHEEL_DIAMETER 0.085
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0
#define POLARITY 0b00111100
#define MOTOR_FR MOTOR1
#define MOTOR_FL MOTOR4
#define MOTOR_RR MOTOR2
#define MOTOR_RL MOTOR3

#endif

namespace rosbot_kinematics {

extern const RosbotDrive_params_t ROSBOT_PARAMS;

typedef struct RosbotOdometry
{
    double wheel_FR_ang_pos;  // radians
    double wheel_FL_ang_pos;  // radians
    double wheel_RR_ang_pos;  // radians
    double wheel_RL_ang_pos;  // radians
    double wheel_L_ang_pos;   // radians
    double wheel_R_ang_pos;   // radians
    double wheel_L_ang_vel;   // radians per second
    double wheel_R_ang_vel;   // radians per second
    double robot_angular_pos; // radians
    double robot_angular_vel; // radians per second
    double robot_x_pos;       // meters
    double robot_y_pos;       // meters
    double robot_x_vel;       // meters per second
    double robot_y_vel;       // meters per second
    RosbotOdometry()
        :wheel_FR_ang_pos(0)
        ,wheel_FL_ang_pos(0)
        ,wheel_RR_ang_pos(0)
        ,wheel_RL_ang_pos(0)
        ,wheel_L_ang_pos(0)
        ,wheel_R_ang_pos(0)
        ,wheel_L_ang_vel(0)
        ,wheel_R_ang_vel(0)
        ,robot_angular_pos(0)
        ,robot_angular_vel(0)
        ,robot_x_pos(0)
        ,robot_y_pos(0)
        ,robot_x_vel(0)
        ,robot_y_vel(0)
        {}
    void reset()
    {
        wheel_FR_ang_pos=0;  // radians
        wheel_FL_ang_pos=0;  // radians
        wheel_RR_ang_pos=0;  // radians
        wheel_RL_ang_pos=0;  // radians
        wheel_L_ang_pos=0;   // radians
        wheel_R_ang_pos=0;   // radians
        wheel_L_ang_vel=0;   // radians per second
        wheel_R_ang_vel=0;   // radians per second
        robot_angular_pos=0; // radians
        robot_angular_vel=0; // radians per second
        robot_x_pos=0;       // meters
        robot_y_pos=0;       // meters
        robot_x_vel=0;       // meters per second
        robot_y_vel=0;       // meters per second
    }
} RosbotOdometry_t;

void setRosbotSpeed(RosbotDrive * drive, float linear, float angular);
void updateRosbotOdometry(RosbotDrive * drive, RosbotOdometry_t * odom, float dtime);
void resetRosbotOdometry(RosbotDrive * drive, RosbotOdometry_t * odom);

}
#endif /* __ROSBOT_KINEMATICS_H__ */