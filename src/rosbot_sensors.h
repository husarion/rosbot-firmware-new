#ifndef __ROSBOT_SENSORS_H__
#define __ROSBOT_SENSORS_H__

#include <mbed.h>
#include <MultiDistanceSensor.h>

void initBatteryWatchdog(events::EventQueue * q, int frequency, float threshold);
float readVoltage();

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

extern const Sensors_pin_def_t SENSORS_PIN_DEF;

#endif /* __ROSBOT_SENSORS_H__ */