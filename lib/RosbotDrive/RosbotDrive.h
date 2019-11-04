/** @file RosbotDrive.h
 * Definitions of RosbotDrive class and helper structures.
 *
 * @date 4.11.2019.
 * @author byq77.
 */
#ifndef __ROSBOT_DRIVE_H__
#define __ROSBOT_DRIVE_H__

#include <mbed.h>
#include "internal/drv88xx-driver-mbed/DRV8848_STM.h"
#include "internal/encoder-mbed/Encoder.h"
#include "internal/rosbot-regulator/RosbotRegulator.h"

#define PWM_DEFAULT_FREQ_HZ 18000UL /**< Default frequency for motors' pwms.*/

/**
 * @brief Rosbot Motor Internal Number.
 * 
 * MOTOR1 and MOTOR2 tags identify driver 1 motor 1 and 2 outputs. 
 * MOTOR3 and MOTOR4 tags identify driver 2 motor 1 and 2 outputs. 
 */
enum RosbotMotNum : uint8_t
{
    MOTOR1 = 0, ///< Driver 1 motor 1
    MOTOR2 = 1, ///< Driver 1 motor 2
    MOTOR3 = 2, ///< Driver 2 motor 1
    MOTOR4 = 3  ///< Driver 2 motor 2
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

struct RosbotWheel
{
    float radius;
    float diameter_modificator;
    float tyre_deflection;
    float gear_ratio;
    uint32_t encoder_cpr; // conuts per revolution
    uint8_t polarity; // LSB -> motor, MSB -> encoder
};

struct NewTargetSpeed
{
    float speed[4];
    SpeedMode mode;
};

struct PidDebugData
{
    float cspeed;
    float tspeed;
    float pidout;
    float error;
};

/**
 * @brief Rosbot Drive Module.
 *
 * This class represents the ROSbot drive module, responsible for motors control, 
 * and odometry calculation. TODO: more description
 * @sa 
 */
class RosbotDrive : NonCopyable<RosbotDrive>
{
public:
    static const RosbotWheel DEFAULT_WHEEL_PARAMS; /**< Default ROSbot's wheels parameters. */

    static const RosbotRegulator_params DEFAULT_REGULATOR_PARAMS; /**< Default ROSbot regulator parameters. */
    
    static RosbotDrive & getInstance(); // done

    void init(const RosbotWheel & wheel_params, const RosbotRegulator_params & params);
    
    void enablePidReg(bool en);

    bool isPidEnabled();

    void enable(bool en=true);
    
    void stop();
    
    float getSpeed(RosbotMotNum mot_num);

    float getSpeed(RosbotMotNum mot_num, SpeedMode mode);

    float getDistance(RosbotMotNum mot_num);

    float getAngularPos(RosbotMotNum mot_num);

    void resetDistance();

    int32_t getEncoderTicks(RosbotMotNum mot_num);

    void updateTargetSpeed(const NewTargetSpeed * new_speed);

    void updateWheelCoefficients(const RosbotWheel & params);

    void updatePidParams(const RosbotRegulator_params * params, bool reset);

    void getPidDebugData(PidDebugData * data, RosbotMotNum mot_num);
    
private:
    static RosbotDrive * _instance;

    RosbotDrive();

    void regulatorLoop();

    volatile RosbotDriveStates _state;
    volatile bool _regulator_output_enabled;
    volatile bool _regulator_loop_enabled;

    RosbotWheel _wheel_params;

    volatile float _tspeed_mps[4];
    volatile float _cspeed_mps[4];
    volatile int32_t _cdistance[4];
    float _pid_interval_s; 

    float _wheel_coefficient1;
    float _wheel_coefficient2;
    
    DRV8848 * _mot_driver[2];
    DRV8848::DRVMotor * _mot[4]; 
    Encoder * _encoder[4];
    RosbotRegulator * _regulator[4];

    Mutex rosbot_drive_mutex;
};

#endif /* __ROSBOT_DRIVE_H__ */