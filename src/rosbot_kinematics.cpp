#include "rosbot_kinematics.h"

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

void updateRosbotOdometry(RosbotDrive * drive, RosbotOdometry_t * odom, float dtime)
{
    if(drive->getRosbotDriveType()==4)
    {
        odom->wheel_FR_ang_pos = drive->getDistance(MOTOR_FR) / WHEEL_RADIUS;
        odom->wheel_FL_ang_pos = drive->getDistance(MOTOR_FL) / WHEEL_RADIUS;
        odom->wheel_RR_ang_pos = drive->getDistance(MOTOR_RR) / WHEEL_RADIUS;
        odom->wheel_RL_ang_pos = drive->getDistance(MOTOR_RL) / WHEEL_RADIUS;

        odom->wheel_L_ang_pos = (odom->wheel_FR_ang_pos + odom->wheel_FL_ang_pos) / (2);
        odom->wheel_R_ang_pos = (odom->wheel_RR_ang_pos + odom->wheel_RL_ang_pos) / (2);
        odom->wheel_L_ang_vel = (drive->getSpeed(MOTOR_FL) + drive->getSpeed(MOTOR_RL)) / (2 * WHEEL_RADIUS);
        odom->wheel_R_ang_vel = (drive->getSpeed(MOTOR_FR) + drive->getSpeed(MOTOR_RR)) / (2 * WHEEL_RADIUS);
    }
    else
    {
        odom->wheel_FR_ang_pos = drive->getDistance(MOTOR_FR) / WHEEL_RADIUS;
        odom->wheel_FL_ang_pos = drive->getDistance(MOTOR_FL) / WHEEL_RADIUS;

        odom->wheel_L_ang_pos = odom->wheel_FL_ang_pos;
        odom->wheel_R_ang_pos = odom->wheel_FR_ang_pos;
        odom->wheel_L_ang_vel = (drive->getSpeed(MOTOR_FL)) / (WHEEL_RADIUS);
        odom->wheel_R_ang_vel = (drive->getSpeed(MOTOR_FR)) / (WHEEL_RADIUS);
    }

    odom->robot_angular_vel = (((odom->wheel_R_ang_pos - odom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR)) - odom->robot_angular_pos) / dtime;
    odom->robot_angular_pos = (odom->wheel_R_ang_pos - odom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR);
    odom->robot_x_vel = (odom->wheel_L_ang_vel * WHEEL_RADIUS + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * cos(odom->robot_angular_pos);
    odom->robot_y_vel = (odom->wheel_L_ang_vel * WHEEL_RADIUS + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * sin(odom->robot_angular_pos);
    odom->robot_x_pos = odom->robot_x_pos + odom->robot_x_vel * dtime;
    odom->robot_y_pos = odom->robot_y_pos + odom->robot_y_vel * dtime;
}

