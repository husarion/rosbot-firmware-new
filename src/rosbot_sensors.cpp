#include "rosbot_sensors.h"

namespace rosbot_sensors{

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

#pragma region IMU_REGION

InterruptIn imu_int(SENS2_PIN1);

Mail<imu_meas_t, 5> imu_sensor_mail_box;

static MPU9250_DMP imu;
static Mutex imu_mutex;

void imuCallback()
{
    imu_mutex.lock();
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() == INV_SUCCESS)
    {
        if(!imu_sensor_mail_box.full())
        {
            imu_meas_t * new_msg = imu_sensor_mail_box.alloc();
            new_msg->qx = imu.calcQuat(imu.qx);   
            new_msg->qy = imu.calcQuat(imu.qy);   
            new_msg->qz = imu.calcQuat(imu.qz);   
            new_msg->qw = imu.calcQuat(imu.qw);
            imu_sensor_mail_box.put(new_msg);
        }
    }
    imu_mutex.unlock();
}

int initImu()
{
    inv_error_t err;
    if ((err = imu.begin()) != INV_SUCCESS)
    {
        return err;
    }
    imu_int.mode(PullUp);
    events::EventQueue *q = mbed_event_queue();
    imu_int.fall(q->event(callback(imuCallback)));

    err = imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT |        // Enable 6-axis quat
                            DMP_FEATURE_GYRO_CAL |      // Use gyro calibration
                            /* DMP_FEATURE_SEND_RAW_ACCEL, */ // Enable raw accel measurements
                        FIFO_SAMPLE_RATE_OPERATION);    // Set DMP FIFO rate

    // err = imu.dmpSetOrientation(MPU_ORIENTATION);
    // The interrupt level can either be active-high or low.
    // Configure as active-low, since we'll be using the pin's
    // internal pull-up resistor.
    // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
    err = imu.setIntLevel(INT_ACTIVE_LOW);

    // The interrupt can be set to latch until data has
    // been read, or to work as a 50us pulse.
    // Use latching method -- we'll read from the sensor
    // as soon as we see the pin go LOW.
    // Options are INT_LATCHED or INT_50US_PULSE
    // Reading any register will clear the interrupt!!!
    err = imu.setIntLatched(INT_LATCHED);

    // Use enableInterrupt() to configure the MPU-9250's
    // interrupt output as a "data ready" indicator.
    err = imu.enableInterrupt(0);
    
    // Disable dmp - it is enabled on demand using enableImu()
    err = imu.dmpState(0);

    return err;
}

void enableImu(bool en)
{
    imu_mutex.lock();
        if(en)
        {
            imu.dmpState(1);
            imu.enableInterrupt(1);
        }
        else
        {
            imu.dmpState(0);
            imu.enableInterrupt(0);
        }
    imu_mutex.unlock();
}
#pragma endregion /* IMU_REGION */

}