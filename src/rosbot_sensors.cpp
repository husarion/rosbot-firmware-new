#include "rosbot_sensors.h"

namespace rosbot_sensors{

#pragma region BATTERY_REGION

#define MEASUREMENT_SERIES 10
#define BATTERY_VOLTAGE_LOW 10.8

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

static BatteryData_t battery_data = { 0.0, BATTERY_VOLTAGE_LOW, BATTERY_OK}; 
static DigitalOut battery_led(LED1,1);
static Ticker battery_led_flipper;
static void readVoltageInternal();
static AnalogIn battery_adc(BAT_MEAS);

static void batteryLed()
{
    battery_led = !battery_led;
}

float updateBatteryWatchdog()
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
    return battery_data.voltage;
}

#pragma endregion BATTERY_REGION

#pragma region IMU_REGION //FIXME mutexes and other improvements

volatile bool imu_state;

const signed char DEFAULT_IMU_ORIENTATION[9] = {
	0, -1, 0,
	-1, 0, 0,
	0, 0, -1
};

InterruptIn imu_int(SENS2_PIN1);

Mail<imu_meas_t, 10> imu_sensor_mail_box;

volatile uint16_t new_data = 0;
static MPU9250_DMP imu;
static Mutex imu_mutex;
Thread imu_thread;

static void imu_interrupt_cb(void)
{
    core_util_atomic_incr_u16(&new_data,1);
}

static void imuCallback()
{
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    imu_mutex.lock();
    // 10.2 Content of DMP Output to FIFO
    // The exact contents of the data output to the FIFO depend on the features that were enabled in Section 6.
    // When all features are enabled, a single set of FIFO data consists of 48 bytes. The data is ordered as shown
    // below:
    // * Low Power 3-Axis Quaternion (16 bytes)
    // * Low Power 6-Axis Quaternion (16 bytes)
    // * Raw Sensor Data (12 bytes)
    // * Gesture Word (Android Orientation + Tap outputs) (4 bytes)
    if(imu.fifoAvailable() >= 32)
    {
        if (imu.dmpUpdateFifo() == INV_SUCCESS)
        {
            if(!imu_sensor_mail_box.full())
            {
                imu_meas_t * new_msg = imu_sensor_mail_box.alloc();
                new_msg->orientation[0] = imu.calcQuat(imu.qx);   
                new_msg->orientation[1] = imu.calcQuat(imu.qy);   
                new_msg->orientation[2] = imu.calcQuat(imu.qz);   
                new_msg->orientation[3] = imu.calcQuat(imu.qw);
                new_msg->angular_velocity[0] = imu.calcGyro(imu.gx);
                new_msg->angular_velocity[1] = imu.calcGyro(imu.gy);
                new_msg->angular_velocity[2] = imu.calcGyro(imu.gz);
                new_msg->linear_velocity[0] = imu.calcAccel(imu.ax);
                new_msg->linear_velocity[1] = imu.calcAccel(imu.ay);
                new_msg->linear_velocity[2] = imu.calcAccel(imu.az);
                new_msg->timestamp = imu.time;
                imu_sensor_mail_box.put(new_msg);
            }
        }
        core_util_atomic_decr_u16(&new_data,1);
    }
    imu_mutex.unlock();
}

static void imu_loop()
{
    while(1)
    {
        if(new_data && imu_state) imuCallback();
        ThisThread::sleep_for(20);
    }
}


int initImu()
{
    inv_error_t err;
    if ((err = imu.begin()) != INV_SUCCESS)
    {
        return err;
    }
    imu_int.mode(PullUp);

    // events::EventQueue *q = mbed_event_queue();
    // imu_int.fall(q->event(callback(imuCallback)));
    imu_int.fall(callback(imu_interrupt_cb));

    err += imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT     | // Enable 6-axis quat
                       DMP_FEATURE_GYRO_CAL       | // Use gyro calibration
                       DMP_FEATURE_SEND_RAW_ACCEL | // Enable raw accel measurements
                       DMP_FEATURE_SEND_CAL_GYRO,   // Enable cal gyro measurements
                       10);                         // Set DMP FIFO rate

    err += imu.dmpSetOrientation(DEFAULT_IMU_ORIENTATION);
    
    err += imu.setGyroFSR(2000); // 2000dps for gyro

    err += imu.setLPF(42); // 42Hz low pass filter
    
    // err = imu.dmpSetOrientation(MPU_ORIENTATION);
    // The interrupt level can either be active-high or low.
    // Configure as active-low, since we'll be using the pin's
    // internal pull-up resistor.
    // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
    err += imu.setIntLevel(INT_ACTIVE_LOW);

    // The interrupt can be set to latch until data has
    // been read, or to work as a 50us pulse.
    // Use latching method -- we'll read from the sensor
    // as soon as we see the pin go LOW.
    // Options are INT_LATCHED or INT_50US_PULSE
    // Reading any register will clear the interrupt!!!
    err += imu.setIntLatched(INT_LATCHED);

    // Use enableInterrupt() to configure the MPU-9250's
    // interrupt output as a "data ready" indicator.
    err += imu.enableInterrupt(1);
    
    // Disable dmp - it is enabled on demand using enableImu()
    // err = imu.dmpState(1);

    imu_thread.start(callback(imu_loop));

    imu_state=true;

    return err;
}

void enableImu(int en)
{
    imu_mutex.lock();
    imu_state=en;
    imu.dmpState(en);
    imu.enableInterrupt(en);
    imu_mutex.unlock();
}

int resetImu()
{
    if(!imu_state)
        return INV_ERROR;
    imu_mutex.lock();
    imu_state = false;
    enableImu(false);
    inv_error_t err;

    if ((err = imu.begin()) != INV_SUCCESS)
    {
        imu_mutex.unlock();
        return err;
    }

    err = imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT     | // Enable 6-axis quat
                       DMP_FEATURE_GYRO_CAL       | // Use gyro calibration
                       DMP_FEATURE_SEND_RAW_ACCEL | // Enable raw accel measurements
                       DMP_FEATURE_SEND_CAL_GYRO,   // Enable raw gyre measurements
                       10);                         // Set DMP FIFO rate

    err = imu.dmpSetOrientation(DEFAULT_IMU_ORIENTATION);
    
    err = imu.setGyroFSR(2000); // 500dps for gyro

    err = imu.setLPF(42); // 42Hz low pass filter

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
    err = imu.enableInterrupt(1);
    
    // Disable dmp - it is enabled on demand using enableImu()
    // err = imu.dmpState(1);
    
    imu_state = true;
    
    imu_mutex.unlock();
    return err;
}

#pragma endregion /* IMU_REGION */

}