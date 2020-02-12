#ifndef __ROSBOT_SENSORS_H__
#define __ROSBOT_SENSORS_H__

#include <mbed.h>
#include <MultiDistanceSensor.h>
#include <SparkFunMPU9250-DMP.h>

#define FIFO_SAMPLE_RATE_OPERATION 10

namespace rosbot_sensors{

typedef struct 
{
    float orientation[4];
    float angular_velocity[3];
    float linear_velocity[3];
    uint32_t timestamp;
}imu_meas_t;

float updateBatteryWatchdog();

extern Mail<imu_meas_t, 10> imu_sensor_mail_box;

int initImu();

int resetImu();

void enableImu(int en);
}

#endif /* __ROSBOT_SENSORS_H__ */