/** @file main.cpp
 * ROSbot firmware - 27th of June 2019 
 */
#include <mbed.h>
#include <RosbotDrive.h>
#include <rosbot_kinematics.h>

const char BUFFER_FORMAT[] = "TIM%d->ARR=%d\r\n" \
                             "TIM%d->PSC=%d\r\n";
char buffer[50];

RosbotDrive * driver;

TIM_TypeDef * test_timers[] = {TIM10,TIM11,TIM13,TIM14};
int index_to_timer[] = {10,11,13,14};
void test()
{
    driver = RosbotDrive::getInstance(&rosbot_kinematics::ROSBOT_PARAMS);
    driver->init();
    int i=0;
    int timers_num = sizeof(index_to_timer)/sizeof(index_to_timer[0]);
    while(1)
    {
        if(i>=timers_num)
            i=0;
        sprintf(buffer,BUFFER_FORMAT,index_to_timer[i],test_timers[i]->ARR,index_to_timer[i],test_timers[i]->PSC);
        puts((const char* )buffer);
        i++;
        wait(1.0);
    }
}