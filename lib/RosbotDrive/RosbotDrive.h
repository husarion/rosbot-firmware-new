#ifndef __ROSBOT_DRIVE_H__
#define __ROSBOT_DRIVE_H__

#include <mbed.h>
#include <arm_math.h>
#include "drv88xx-driver-mbed/DRV8848_STM.h"
#include "encoder-mbed/Encoder.h"
#define PWM_DEFAULT_FREQ_HZ 18000UL

enum RosbotMotNum
{
    MOTOR1 = 0,
    MOTOR2 = 1,
    MOTOR3 = 2,
    MOTOR4 = 3
};

enum SpeedMode
{
    TICSKPS,
    MPS,
    DUTY_CYCLE
};

enum RosbotDriveStates
{
    UNINIT,
    HALT,
    IDLE,
    OPERATIONAL,
    FAULT
};

typedef struct RosobtDrivePid
{
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float a_max;
    float da_max;
    float speed_max;
    uint32_t dt_ms;
}RosbotDrivePid_t;

typedef struct RosbotWheel
{
    float radius;
    float diameter_modificator;
    float tyre_deflection;
    float gear_ratio;
    uint32_t encoder_cpr; // conuts per revolution
}RosbotWheel_t;

typedef struct RosbotDriveParams
{
    RosbotDrivePid_t pid_params;
    RosbotWheel_t wheel_params;
    uint8_t polarity; // LSB -> motor, MSB -> enkoder
}RosbotDrive_params_t;

typedef struct NewTargetSpeed
{
    float speed[4];
    SpeedMode mode;
    NewTargetSpeed()
    :speed{0,0,0,0},mode(MPS)
    {}
}NewTargetSpeed_t;

typedef struct PidDebugData
{
    float cspeed;
    float tspeed;
    float pidout;
    float error;
}PidDebugData_t;

class RosbotDrive
{
public:
    static const RosbotWheel_t DEFAULT_WHEEL_PARAMS;
    static const RosbotDrivePid_t DEFAULT_PID_PARAMS;
    static RosbotDrive * getInstance(const RosbotDrive_params_t * params=NULL);
    static int getRosbotDriveType();
    void init(void);
    void enable(bool en=true);
    void stop();
    void enablePidReg(bool en);
    bool isPidEnabled();
    float getSpeed(RosbotMotNum mot_num);
    float getSpeed(RosbotMotNum mot_num, SpeedMode mode);
    float getDistance(RosbotMotNum mot_num);
    float getAngularPos(RosbotMotNum mot_num);
    void resetDistance();
    int32_t getEncoderTicks(RosbotMotNum mot_num);
    void updateTargetSpeed(const NewTargetSpeed_t * new_speed);
    void updateWheelParams(const RosbotWheel_t * params);
    void updatePidParams(const RosbotDrivePid_t * params, bool reset);
    void getPidDebugData(PidDebugData_t * data, RosbotMotNum mot_num);
    
private:
    static RosbotDrive * _instance;
    void regulatorLoop();
    RosbotDrive(const RosbotDrive_params_t * params);
    volatile RosbotDriveStates _state;
    volatile bool _pid_state;
    volatile bool _regulator_state;
    RosbotDrivePid_t _pid_params;
    uint8_t _polarity;
    volatile float _tspeed_mps[4];
    volatile float _cspeed_mps[4];
    volatile int32_t _cdistance[4];
    volatile float _error[4];
    volatile float _pidout[4];
    double _wheel_coefficient1;
    double _wheel_coefficient2;
    float _pid_interval_s;
    DRV8848 * _mot_driver[2];
    DRV8848::DRVMotor * _mot[4]; 
    Encoder * _encoder[4];
    arm_pid_instance_f32 * _pid_instance[4];
    Mutex rosbot_drive_mutex;
};

#endif /* __ROSBOT_DRIVE_H__ */