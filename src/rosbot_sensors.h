#ifndef __ROSBOT_SENSORS_H__
#define __ROSBOT_SENSORS_H__

#include <mbed.h>
#include <MultiDistanceSensor.h>
#include <SparkFunMPU9250-DMP.h>

#define FIFO_SAMPLE_RATE_OPERATION 10

#define SENSOR_FR 0
#define SENSOR_FL 1
#define SENSOR_RR 2
#define SENSOR_RL 3

#define SENSOR_FR_XSHOUT_PIN SENS6_PIN1
#define SENSOR_FL_XSHOUT_PIN SENS6_PIN2 
#define SENSOR_RR_XSHOUT_PIN SENS6_PIN4
#define SENSOR_RL_XSHOUT_PIN SENS6_PIN3 
#define SENSORS_SDA_PIN SENS1_PIN4
#define SENSORS_SCL_PIN SENS1_PIN3

namespace rosbot_sensors{

extern const Sensors_pin_def_t SENSORS_PIN_DEF;

typedef struct 
{
    float orientation[4];
    float angular_velocity[3];
    float linear_velocity[3];
}imu_meas_t;

float updateBatteryWatchdog();

extern Mail<imu_meas_t, 10> imu_sensor_mail_box;

int initImu();

int resetImu();

void enableImu(int en);
}

#endif /* __ROSBOT_SENSORS_H__ */