#include "MultiDistanceSensor.h"

static const uint8_t DEFAULT_HW_ADDRESS = 0x29;

static const uint8_t SENSOR_HW_ADDRESS[]={
    DEFAULT_HW_ADDRESS + 1,
    DEFAULT_HW_ADDRESS + 2,
    DEFAULT_HW_ADDRESS + 3,
    DEFAULT_HW_ADDRESS + 4,
};

#if defined(TARGET_CORE2)

#define SENSOR_FR_XSHOUT_PIN SENS6_PIN1
#define SENSOR_FL_XSHOUT_PIN SENS6_PIN2 
#define SENSOR_RR_XSHOUT_PIN SENS6_PIN4
#define SENSOR_RL_XSHOUT_PIN SENS6_PIN3 
#define SENSORS_SDA_PIN SENS1_PIN4
#define SENSORS_SCL_PIN SENS1_PIN3

static DigitalInOut xshout1(SENSOR_FR_XSHOUT_PIN, PIN_OUTPUT, OpenDrainNoPull, 0);
static DigitalInOut xshout2(SENSOR_FL_XSHOUT_PIN, PIN_OUTPUT, OpenDrainNoPull, 0);
static DigitalInOut xshout3(SENSOR_RR_XSHOUT_PIN, PIN_OUTPUT, OpenDrainNoPull, 0);
static DigitalInOut xshout4(SENSOR_RL_XSHOUT_PIN, PIN_OUTPUT, OpenDrainNoPull, 0);
static I2C i2c(SENSORS_SDA_PIN,SENSORS_SCL_PIN);

#else
    #error "Your target is not supported!"
#endif /* TARGET_CORE2 */

MultiDistanceSensor * MultiDistanceSensor::_instance = nullptr;

Mail<SensorsMeasurement, 5> distance_sensor_mail_box;
Mail<uint8_t, 5> distance_sensor_commands;

MultiDistanceSensor::MultiDistanceSensor()
:_i2c(&i2c)
,_xshout{&xshout1, &xshout2, &xshout3, &xshout4}
,_is_active{false,false,false,false}
{
    for(int i=0;i<NUM_DISTANCE_SENSORS;i++)
    {
        _sensor[i] = new VL53L0X(*_i2c);
    }
}

MultiDistanceSensor & MultiDistanceSensor::getInstance()
{
    if(_instance==NULL)
    {
        static MultiDistanceSensor instance;
        _instance = &instance;
    }
    return *_instance;
}

MultiDistanceSensor::~MultiDistanceSensor()
{
    stop();
    for(int i=0;i<4;i++) delete _sensor[i];
}

bool MultiDistanceSensor::restart()
{
    int result=0;

    for(int i=0;i<4;i++)
    {
        _xshout[i]->write(0);
        _sensor[i]->setDefaultAddress();
        _is_active[i] = false;
    }

    ThisThread::sleep_for(10);

    for(int i=0;i<4;i++)
    {
        _sensor[i]->setTimeout(500);
        _xshout[i]->write(1);
        ThisThread::sleep_for(10);
        if(_sensor[i]->init())
        {
            _sensor[i]->setAddress(SENSOR_HW_ADDRESS[i]);
            // _sensor[i]->setMeasurementTimingBudget(100);
            _is_active[i]=true;
            result++;
        }
    }

    return result == NUM_DISTANCE_SENSORS;
}

void MultiDistanceSensor::stop()
{
    for(int i=0;i<NUM_DISTANCE_SENSORS;i++)
    {
        if(_is_active[i])
        {
            _sensor[i]->stopContinuous();
        }
    }
    _sensors_enabled = false;
}

void MultiDistanceSensor::start()
{
    for(int i=0;i<NUM_DISTANCE_SENSORS;i++)
    {
        if(_is_active[i])
        {
            // _sensor[i]->setTimeout(100);
            _sensor[i]->startContinuous(100);
        }
    }
    _sensors_enabled = true;
}

bool MultiDistanceSensor::init()
{
    if(_initialized)
        return false;
    _i2c->frequency(DISTANCE_SENSORS_DEFAULT_I2C_FREQ);
    if(restart())
    {
        _initialized = true;
        start();
        _distance_sensor_thread.start(callback(this,&MultiDistanceSensor::sensors_loop));
        return true;
    }
    else
        return false;
}

void MultiDistanceSensor::sensors_loop()
{
    while (1)
    {
        osEvent evt = distance_sensor_commands.get(0);

        if(evt.status == osEventMail)
        {
            uint8_t * command = (uint8_t *)evt.value.p;
            switch(*command)
            {
                case 0:
                    stop();
                    break;
                case 1:
                    start();
                    break;
                case 2:
                    i2c.abort_transfer();
                    restart();
                    break;
                default:
                    break;
            }
            distance_sensor_commands.free(command);
        }

        if (_sensors_enabled) runMeasurement();
        
        ThisThread::sleep_for(15);
    }
}

int MultiDistanceSensor::runMeasurement()
{
    int result = ERR_NONE;
    SensorsMeasurement * msg;
    bool is_measurement_ready = (_sensor[LAST_SENSOR_INDEX]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07); 
    
    if (_sensor[LAST_SENSOR_INDEX]->last_status != 0)
    {
        if (!distance_sensor_mail_box.full())
        {
            msg = distance_sensor_mail_box.alloc();

            if (msg == nullptr)
                return ERR_BUSSY;

            msg->status = ERR_I2C_FAILURE;

            for(int i=0;i<4;i++)
            {
                msg->range[i] = -1.0;
                _is_active[i] = false;
            }
            distance_sensor_mail_box.put(msg);
        }
        result = ERR_I2C_FAILURE;
    }
    else if(!is_measurement_ready)
    {
        return ERR_NOT_READY;
    }
    else if (!distance_sensor_mail_box.full())
    {
        msg = distance_sensor_mail_box.alloc();
        if (msg == nullptr)
            return ERR_BUSSY;
        
        msg->timestamp = Kernel::get_ms_count();
        msg->status = ERR_NONE;

        for(int i=0; i<NUM_DISTANCE_SENSORS; i++)
        {
            if(_is_active[i])
            {
                uint16_t range = _sensor[i]->readRangeContinuousMillimeters(false);
                if(_sensor[i]->last_status == 0)
                {
                    msg->range[i] = (float) range / 1000.0;
                }
                else
                {
                    msg->range[i] = -1.0;
                    _is_active[i] = false;
                    result = ERR_I2C_FAILURE;
                    msg->status = ERR_I2C_FAILURE;
                }
            }
        }
        distance_sensor_mail_box.put(msg);
    }
    return result;
}