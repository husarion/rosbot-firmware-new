#include "rosbot_kinematics.h"

namespace rosbot_kinematics {

const RosbotWheel CUSTOM_WHEEL_PARAMS = {
    .radius = WHEEL_RADIUS,
    .diameter_modificator = DIAMETER_MODIFICATOR,
    .tyre_deflection = TYRE_DEFLATION,
    .gear_ratio = GEAR_RATIO,
    .encoder_cpr = ENCODER_CPR,
    .polarity = POLARITY
};

void setRosbotSpeed(RosbotDrive & drive, float linear, float angular)
{
    NewTargetSpeed new_speed;
    new_speed.mode = MPS;
    new_speed.speed[MOTOR_FL] = new_speed.speed[MOTOR_RL] = linear - (angular * ROBOT_WIDTH_HALF);
    new_speed.speed[MOTOR_FR] = new_speed.speed[MOTOR_RR] = linear + (angular * ROBOT_WIDTH_HALF);
    drive.updateTargetSpeed(new_speed);
}

void updateRosbotOdometry(RosbotDrive & drive, RosbotOdometry & odom, float dtime)
{
    float curr_wheel_R_ang_pos;
    float curr_wheel_L_ang_pos;
    Odometry * iodom = &odom.odom;
    iodom->wheel_FR_ang_pos = drive.getAngularPos(MOTOR_FR);
    iodom->wheel_FL_ang_pos = drive.getAngularPos(MOTOR_FL);
    iodom->wheel_RR_ang_pos = drive.getAngularPos(MOTOR_RR);
    iodom->wheel_RL_ang_pos = drive.getAngularPos(MOTOR_RL);
    curr_wheel_R_ang_pos = (iodom->wheel_FR_ang_pos + iodom->wheel_RR_ang_pos)/(2*TYRE_DEFLATION);
    curr_wheel_L_ang_pos = (iodom->wheel_FL_ang_pos + iodom->wheel_RL_ang_pos)/(2*TYRE_DEFLATION);
    iodom->wheel_L_ang_vel = (curr_wheel_L_ang_pos - iodom->wheel_L_ang_pos) / (dtime);
    iodom->wheel_R_ang_vel = (curr_wheel_R_ang_pos - iodom->wheel_R_ang_pos) / (dtime);
    iodom->wheel_L_ang_pos = curr_wheel_L_ang_pos;
    iodom->wheel_R_ang_pos = curr_wheel_R_ang_pos;
    iodom->robot_angular_vel = (((iodom->wheel_R_ang_pos - iodom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR)) - iodom->robot_angular_pos) / dtime;
    iodom->robot_angular_pos = (iodom->wheel_R_ang_pos - iodom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR);
    iodom->robot_x_vel = (iodom->wheel_L_ang_vel * WHEEL_RADIUS + iodom->robot_angular_vel * ROBOT_WIDTH_HALF) * cos(iodom->robot_angular_pos);
    iodom->robot_y_vel = (iodom->wheel_L_ang_vel * WHEEL_RADIUS + iodom->robot_angular_vel * ROBOT_WIDTH_HALF) * sin(iodom->robot_angular_pos);
    iodom->robot_x_pos = iodom->robot_x_pos + iodom->robot_x_vel * dtime;
    iodom->robot_y_pos = iodom->robot_y_pos + iodom->robot_y_vel * dtime;
}

void resetRosbotOdometry(RosbotDrive & drive, RosbotOdometry & odom)
{
    drive.enablePidReg(0);
    memset(odom.buffor,0,sizeof(odom.buffor));
    drive.resetDistance();
    drive.enablePidReg(1);
}

}