#include "rosbot_kinematics.h"

namespace rosbot_kinematics {

const RosbotDrive_params_t ROSBOT_PARAMS = {
    .pid_params = RosbotDrive::DEFAULT_PID_PARAMS,
    .wheel_params = {
        .radius = WHEEL_RADIUS,
        .diameter_modificator = DIAMETER_MODIFICATOR,
        .tyre_deflection = TYRE_DEFLATION,
        .gear_ratio = GEAR_RATIO,
        .encoder_cpr = ENCODER_CPR},
    .polarity = POLARITY
};

void setRosbotSpeed(RosbotDrive * drive, float linear, float angular)
{
    static NewTargetSpeed_t new_speed;
    new_speed.speed[MOTOR_FL] = new_speed.speed[MOTOR_RL] = linear - (angular * ROBOT_WIDTH_HALF);
    new_speed.speed[MOTOR_FR] = new_speed.speed[MOTOR_RR] = linear + (angular * ROBOT_WIDTH_HALF);
    drive->updateTargetSpeed(&new_speed);
}

//TODO: fix odometry

void updateRosbotOdometry(RosbotDrive * drive, RosbotOdometry_t * odom, float dtime)
{
    double curr_wheel_R_ang_pos;
    double curr_wheel_L_ang_pos;
    odom->wheel_FR_ang_pos = drive->getAngularPos(MOTOR_FR);
    odom->wheel_FL_ang_pos = drive->getAngularPos(MOTOR_FL);
    odom->wheel_RR_ang_pos = drive->getAngularPos(MOTOR_RR);
    odom->wheel_RL_ang_pos = drive->getAngularPos(MOTOR_RL);
    if(drive->getRosbotDriveType()==4)
    {
#if 0
        // Error is too large in this method
        double curr_wheel_L_lin_vel = (drive->getSpeed(MOTOR_FL)+drive->getSpeed(MOTOR_RL))/2;
        double curr_wheel_R_lin_vel = (drive->getSpeed(MOTOR_FR)+drive->getSpeed(MOTOR_RR))/2;
        odom->robot_angular_vel = (curr_wheel_R_lin_vel - curr_wheel_L_lin_vel)/ (ROBOT_WIDTH * DIAMETER_MODIFICATOR);
        odom->robot_angular_pos = odom->robot_angular_pos + odom->robot_angular_vel * (dtime);
        odom->robot_x_vel = (curr_wheel_L_lin_vel + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * cos(odom->robot_angular_pos); 
        odom->robot_y_vel = (curr_wheel_L_lin_vel + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * sin(odom->robot_angular_pos); 
#endif
        curr_wheel_R_ang_pos = (odom->wheel_FR_ang_pos + odom->wheel_RR_ang_pos)/(2*TYRE_DEFLATION);
        curr_wheel_L_ang_pos = (odom->wheel_FL_ang_pos + odom->wheel_RL_ang_pos)/(2*TYRE_DEFLATION);
    }
    else
    {
        curr_wheel_R_ang_pos = odom->wheel_FR_ang_pos;
        curr_wheel_L_ang_pos = odom->wheel_FL_ang_pos;
    }
    odom->wheel_L_ang_vel = (curr_wheel_L_ang_pos - odom->wheel_L_ang_pos) / (dtime);
    odom->wheel_R_ang_vel = (curr_wheel_R_ang_pos - odom->wheel_R_ang_pos) / (dtime);
    odom->wheel_L_ang_pos = curr_wheel_L_ang_pos;
    odom->wheel_R_ang_pos = curr_wheel_R_ang_pos;
    odom->robot_angular_vel = (((odom->wheel_R_ang_pos - odom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR)) - odom->robot_angular_pos) / dtime;
    odom->robot_angular_pos = (odom->wheel_R_ang_pos - odom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR);
    odom->robot_x_vel = (odom->wheel_L_ang_vel * WHEEL_RADIUS + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * cos(odom->robot_angular_pos);
    odom->robot_y_vel = (odom->wheel_L_ang_vel * WHEEL_RADIUS + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * sin(odom->robot_angular_pos);
    odom->robot_x_pos = odom->robot_x_pos + odom->robot_x_vel * dtime;
    odom->robot_y_pos = odom->robot_y_pos + odom->robot_y_vel * dtime;
}

void resetRosbotOdometry(RosbotDrive * drive, RosbotOdometry_t * odom)
{
    drive->enablePidReg(0);
    drive->resetDistance();
    odom->reset();
    drive->enablePidReg(1);
}

}