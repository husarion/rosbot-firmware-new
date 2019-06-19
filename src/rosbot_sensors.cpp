#include "rosbot_sensors.h"

#pragma region BATTERY_REGION

#define MEASUREMENT_SERIES 10

enum 
{
    BATTERY_LOW = 1,
    BATTERY_OK = 0
};

typedef struct BatteryData
{
    float voltage;
    float threshold;
    uint8_t status;
}BatteryData_t;

static BatteryData_t battery_data; 
static DigitalOut battery_led(LED1,1);
static Ticker battery_led_flipper;
static void readVoltageInternal();
static AnalogIn battery_adc(BAT_MEAS);

static void batteryLed()
{
    battery_led = !battery_led;
}

float readVoltage()
{
    return battery_data.voltage;    
}

static void readVoltageInternal()
{
    static int index=0;
    battery_data.voltage = 3.3f * VIN_MEAS_CORRECTION * (UPPER_RESISTOR + LOWER_RESISTOR)/LOWER_RESISTOR * battery_adc.read();
    if(battery_data.threshold > battery_data.voltage && index < MEASUREMENT_SERIES) // low level
        index ++;
    else if(battery_data.threshold < battery_data.voltage && index > 0)
        index --;

    if(battery_data.status == BATTERY_OK && index == MEASUREMENT_SERIES)
    {
        battery_data.status = BATTERY_LOW;
        battery_led = 0;
        battery_led_flipper.attach(callback(batteryLed),0.4);
    }
    else if(battery_data.status == BATTERY_LOW && index == 0)
    {
        battery_data.status = BATTERY_OK;
        battery_led_flipper.detach();
        battery_led = 1;
    }
}

void initBatteryWatchdog(events::EventQueue * q, int frequency, float threshold)
{
    if(q==NULL)
        return;
    battery_data.threshold = threshold;
    battery_data.status = BATTERY_OK;
    q->call_every(1000.0/frequency,callback(readVoltageInternal));
}

#pragma endregion BATTERY_REGION

#pragma region DISTANCE_SENSOR_REGION

const Sensors_pin_def_t SENSORS_PIN_DEF={
    .sda_pin=SENSORS_SDA_PIN,
    .scl_pin=SENSORS_SCL_PIN,
    .xshout_sens_pin={
        SENSOR_FR_XSHOUT_PIN,
        SENSOR_FL_XSHOUT_PIN,
        SENSOR_RR_XSHOUT_PIN,
        SENSOR_RL_XSHOUT_PIN}
};

#pragma endregion DISTANCE_SENSOR_REGION