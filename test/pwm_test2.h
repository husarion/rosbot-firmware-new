/** @file main.cpp
 * ROSbot firmware - 27th of June 2019 
 */
#include <mbed.h>
#include <RosbotDrive.h>
#include <rosbot_kinematics.h>

RosbotDrive * driver;

DigitalOut led(LED1);

void test()
{
    driver = RosbotDrive::getInstance(&rosbot_kinematics::ROSBOT_PARAMS);
    driver->init();
    driver->enable();
    NewTargetSpeed_t t;
    t.mode = DUTY_CYCLE;
    float test_value = 0.0;
    while(1)
    {
        t.speed[0] = sin(test_value+=0.1f);
        // driver->updateTargetSpeed(&t);
        printf("TIM10->CCR1 = %d\r\n",TIM10->CCR1);
        led = !led;
        wait_ms(200);
    }
}