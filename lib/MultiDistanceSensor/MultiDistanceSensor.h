#ifndef __MULTI_DISTANCE_SENSOR_H__
#define __MULTI_DISTANCE_SENSOR_H__

#include "vl53l0x-mbed/VL53L0X.h"

typedef struct Sensors_pin_def
{
    PinName sda_pin;
    PinName scl_pin;
    PinName xshout_sens_pin[4];
} Sensors_pin_def_t;

typedef struct SensorsMeasurement
{
    float range[4];
} SensorsMeasurement_t;

class MultiDistanceSensor
{
public:
    int init(int freq=100000);
    void shutdownSensors();
    VL53L0X *getSensor(int num);
    static MultiDistanceSensor *getInstance(const Sensors_pin_def_t *pin_def = NULL);

private:
    static MultiDistanceSensor * _instance;
    MultiDistanceSensor(const Sensors_pin_def_t *pin_def);
    ~MultiDistanceSensor();
    I2C *_i2c;
    Timer *_t;
    DigitalInOut *_xshout[4];
    VL53L0X *_sensors[4];
};

#endif