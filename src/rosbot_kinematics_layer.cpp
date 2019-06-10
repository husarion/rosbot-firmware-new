#include "rosbot_kinematics_layer.h"

const RosbotDrive_params_t ROSBOT_PARAMS = {
    .pid_params = RosbotDrive::DEFAULT_PID_PARAMS,
    .wheel_params = {
        .radius = WHEEL_RADIUS,
        .diameter_modificator = DIAMETER_MODIFICATOR,
        .tyre_deflection = TYRE_DEFLACTION,
        .gear_ratio = GEAR_RATIO,
        .encoder_cpr = ENCODER_CPR},
    .polarity = POLARITY
};

void setRosbotSpeed(RosbotDrive * drive, float linear, float angular)
{
    static NewTargetSpeed_t new_speed;
    new_speed.speed[FL] = new_speed.speed[RL] = linear - (angular * ROBOT_WIDTH_HALF);
    new_speed.speed[FR] = new_speed.speed[RR] = linear + (angular * ROBOT_WIDTH_HALF);
    drive->updateTargetSpeed(&new_speed);
}

void updateRosbotOdometry(RosbotDrive * drive, RosbotOdometry_t * odom, float dtime)
{
    if(drive->getRosbotDriveType()==4)
    {
        odom->wheel_FR_ang_pos = drive->getDistance(FR) / WHEEL_RADIUS;
        odom->wheel_FL_ang_pos = drive->getDistance(FL) / WHEEL_RADIUS;
        odom->wheel_RR_ang_pos = drive->getDistance(RR) / WHEEL_RADIUS;
        odom->wheel_RL_ang_pos = drive->getDistance(RL) / WHEEL_RADIUS;

        odom->wheel_L_ang_pos = (odom->wheel_FR_ang_pos + odom->wheel_FL_ang_pos) / (2);
        odom->wheel_R_ang_pos = (odom->wheel_RR_ang_pos + odom->wheel_RL_ang_pos) / (2);
        odom->wheel_L_ang_vel = (drive->getSpeed(FL) + drive->getSpeed(RL)) / (2 * WHEEL_RADIUS);
        odom->wheel_R_ang_vel = (drive->getSpeed(FR) + drive->getSpeed(RR)) / (2 * WHEEL_RADIUS);
    }
    else
    {
        odom->wheel_FR_ang_pos = drive->getDistance(FR) / WHEEL_RADIUS;
        odom->wheel_FL_ang_pos = drive->getDistance(FL) / WHEEL_RADIUS;

        odom->wheel_L_ang_pos = odom->wheel_FL_ang_pos;
        odom->wheel_R_ang_pos = odom->wheel_FR_ang_pos;
        odom->wheel_L_ang_vel = (drive->getSpeed(FL)) / (WHEEL_RADIUS);
        odom->wheel_R_ang_vel = (drive->getSpeed(FR)) / (WHEEL_RADIUS);
    }

    odom->robot_angular_vel = (((odom->wheel_R_ang_pos - odom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR)) - odom->robot_angular_pos) / dtime;
    odom->robot_angular_pos = (odom->wheel_R_ang_pos - odom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * DIAMETER_MODIFICATOR);
    odom->robot_x_vel = (odom->wheel_L_ang_vel * WHEEL_RADIUS + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * cos(odom->robot_angular_pos);
    odom->robot_y_vel = (odom->wheel_L_ang_vel * WHEEL_RADIUS + odom->robot_angular_vel * ROBOT_WIDTH_HALF) * sin(odom->robot_angular_pos);
    odom->robot_x_pos = odom->robot_x_pos + odom->robot_x_vel * dtime;
    odom->robot_y_pos = odom->robot_y_pos + odom->robot_y_vel * dtime;
}

