#include <mbed.h>
#include <rosbot_sensors.h>

DigitalOut sens_power(SENS_POWER_ON,0);

int test()
{
    ThisThread::sleep_for(100);
    sens_power=1;
    printf("Start\r\n");
    rosbot_sensors::initImu();

    while(1)
    {
        osEvent evt = rosbot_sensors::imu_sensor_mail_box.get(0);

        if(evt.status == osEventMail)
        {
            rosbot_sensors::imu_meas_t * message = (rosbot_sensors::imu_meas_t*)evt.value.p;

            printf("New message\r\n");
            rosbot_sensors::imu_sensor_mail_box.free(message);
        }
        ThisThread::sleep_for(10);
    }
}