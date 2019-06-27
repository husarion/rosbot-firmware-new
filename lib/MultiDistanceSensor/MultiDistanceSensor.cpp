#include "MultiDistanceSensor.h"

static const uint8_t DEFAULT_HW_ADDRESS = 0x29;

static const uint8_t SENSOR_HW_ADDRESS[]={
    DEFAULT_HW_ADDRESS + 1,
    DEFAULT_HW_ADDRESS + 2,
    DEFAULT_HW_ADDRESS + 3,
    DEFAULT_HW_ADDRESS + 4,
};

MultiDistanceSensor * MultiDistanceSensor::_instance=NULL;

MultiDistanceSensor::MultiDistanceSensor(const Sensors_pin_def_t * pin_def)
{
    MBED_ASSERT(pin_def);
    _i2c = new I2C(pin_def->sda_pin,pin_def->scl_pin);
    _t = new Timer;
    for(int i=0;i<4;i++)
    {
        _xshout[i] = new DigitalInOut(pin_def->xshout_sens_pin[i], PIN_OUTPUT, OpenDrainNoPull, 0);
        _sensors[i] = new VL53L0X(*_i2c,*_t);
    }
}

MultiDistanceSensor * MultiDistanceSensor::getInstance(const Sensors_pin_def_t * pin_def)
{
    if(_instance==NULL && pin_def!=NULL)
    {
        _instance = new MultiDistanceSensor(pin_def);
    }
    return _instance;
}

void MultiDistanceSensor::shutdownSensors(){};

MultiDistanceSensor::~MultiDistanceSensor()
{
    shutdownSensors();
    for(int i=0;i<4;i++)
    {
        delete _xshout[i];
        delete _sensors[i];
    }
    delete _i2c;
    delete _t;
}

int MultiDistanceSensor::init(int freq)
{
    int result=0;
    _i2c->frequency(freq);
    for(int i=0;i<4;i++)
    {
        _sensors[i]->setTimeout(500);
        _xshout[i]->write(1);
        wait_ms(10);
        if(_sensors[i]->init())
        {
            _sensors[i]->setAddress(SENSOR_HW_ADDRESS[i]);
            result++;
        }
    }
    return result;
}

VL53L0X * MultiDistanceSensor::getSensor(int num)
{
    return (num >= 0 && num <= 3) ? _sensors[num] : NULL;
}