#include "rosbot_kinematics.h"

namespace rosbot_kinematics
{

    RosbotWheel custom_wheel_params = {
        .radius = WHEEL_RADIUS,
        .diameter_modificator = DIAMETER_MODIFICATOR,
        .tyre_deflation = TYRE_DEFLATION,
        .gear_ratio = GEAR_RATIO,
        .encoder_cpr = ENCODER_CPR,
        .polarity = POLARITY};

    RosbotKinematics::RosbotKinematics()
    {
    }

    RosbotKinematics::~RosbotKinematics()
    {
    }

    void RosbotKinematics::resetRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom)
    {
        drive.enablePidReg(0);
        memset(odom.buffor, 0, sizeof(odom.buffor));
        drive.resetDistance();
        drive.enablePidReg(1);
    }

    void RosbotKinematics::calibrateOdometry(float diameter_modificator, float tyre_deflation)
    {
        custom_wheel_params.diameter_modificator = diameter_modificator;
        custom_wheel_params.tyre_deflation = tyre_deflation;
    }

    RosbotKinematics *RosbotKinematics::kinematicsType(int type)
    {
        if (type == 0)
            return new DifferentialDrive;
        else if (type == 1)
            return new MecanumDrive;
        else
            return new DifferentialDrive;
    }

    MecanumDrive::MecanumDrive()
    {
    }

    MecanumDrive::~MecanumDrive()
    {
    }

    DifferentialDrive::DifferentialDrive()
    {
    }

    DifferentialDrive::~DifferentialDrive()
    {
    }

    void DifferentialDrive::updateRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom, float dtime)
    {
        float curr_wheel_R_ang_pos;
        float curr_wheel_L_ang_pos;
        Odometry *iodom = &odom.odom;
        iodom->wheel_FR_ang_pos = drive.getAngularPos(MOTOR_FR);
        iodom->wheel_FL_ang_pos = drive.getAngularPos(MOTOR_FL);
        iodom->wheel_RR_ang_pos = drive.getAngularPos(MOTOR_RR);
        iodom->wheel_RL_ang_pos = drive.getAngularPos(MOTOR_RL);
        curr_wheel_R_ang_pos = (iodom->wheel_FR_ang_pos + iodom->wheel_RR_ang_pos) / (2 * custom_wheel_params.tyre_deflation);
        curr_wheel_L_ang_pos = (iodom->wheel_FL_ang_pos + iodom->wheel_RL_ang_pos) / (2 * custom_wheel_params.tyre_deflation);
        iodom->wheel_L_ang_vel = (curr_wheel_L_ang_pos - iodom->wheel_L_ang_pos) / (dtime);
        iodom->wheel_R_ang_vel = (curr_wheel_R_ang_pos - iodom->wheel_R_ang_pos) / (dtime);
        iodom->wheel_L_ang_pos = curr_wheel_L_ang_pos;
        iodom->wheel_R_ang_pos = curr_wheel_R_ang_pos;
        iodom->robot_angular_vel = (((iodom->wheel_R_ang_pos - iodom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * custom_wheel_params.diameter_modificator)) - iodom->robot_angular_pos) / dtime;
        iodom->robot_angular_pos = (iodom->wheel_R_ang_pos - iodom->wheel_L_ang_pos) * WHEEL_RADIUS / (ROBOT_WIDTH * custom_wheel_params.diameter_modificator);
        iodom->robot_x_vel = (iodom->wheel_L_ang_vel * WHEEL_RADIUS + iodom->robot_angular_vel * ROBOT_WIDTH_HALF) * cos(iodom->robot_angular_pos);
        iodom->robot_y_vel = (iodom->wheel_L_ang_vel * WHEEL_RADIUS + iodom->robot_angular_vel * ROBOT_WIDTH_HALF) * sin(iodom->robot_angular_pos);
        iodom->robot_x_pos = iodom->robot_x_pos + iodom->robot_x_vel * dtime;
        iodom->robot_y_pos = iodom->robot_y_pos + iodom->robot_y_vel * dtime;
    }

    void DifferentialDrive::setRosbotSpeed(RosbotDrive &drive, RosbotSpeed &speed)
    {
        NewTargetSpeed new_speed;
        new_speed.mode = MPS;
        new_speed.speed[MOTOR_FL] = new_speed.speed[MOTOR_RL] = speed.lin_x - (speed.ang_z * ROBOT_WIDTH_HALF);
        new_speed.speed[MOTOR_FR] = new_speed.speed[MOTOR_RR] = speed.lin_x + (speed.ang_z * ROBOT_WIDTH_HALF);
        drive.updateTargetSpeed(new_speed);
    }

    void DifferentialDrive::setRosbotSpeed(RosbotDrive &drive, float linear_x, float angular_z)
    {
        NewTargetSpeed new_speed;
        new_speed.mode = MPS;
        new_speed.speed[MOTOR_FL] = new_speed.speed[MOTOR_RL] = linear_x - (angular_z * ROBOT_WIDTH_HALF);
        new_speed.speed[MOTOR_FR] = new_speed.speed[MOTOR_RR] = linear_x + (angular_z * ROBOT_WIDTH_HALF);
        drive.updateTargetSpeed(new_speed);
    }

    void MecanumDrive::setRosbotSpeed(RosbotDrive &drive, RosbotSpeed &speed)
    {
        NewTargetSpeed new_speed;
        new_speed.mode = MPS;
        new_speed.speed[MOTOR_FR] = (1 / custom_wheel_params.radius) * (speed.lin_x + speed.lin_y + (ROBOT_WIDTH + ROBOT_LENGTH) * speed.ang_z); //  # rad/s
        new_speed.speed[MOTOR_FL] = (1 / custom_wheel_params.radius) * (speed.lin_x - speed.lin_y - (ROBOT_WIDTH + ROBOT_LENGTH) * speed.ang_z);
        new_speed.speed[MOTOR_RR] = (1 / custom_wheel_params.radius) * (speed.lin_x - speed.lin_y + (ROBOT_WIDTH + ROBOT_LENGTH) * speed.ang_z);
        new_speed.speed[MOTOR_RL] = (1 / custom_wheel_params.radius) * (speed.lin_x + speed.lin_y - (ROBOT_WIDTH + ROBOT_LENGTH) * speed.ang_z);
    }

    void MecanumDrive::updateRosbotOdometry(RosbotDrive &drive, RosbotOdometry &odom, float dtime)
    {
    }

} // namespace rosbot_kinematics