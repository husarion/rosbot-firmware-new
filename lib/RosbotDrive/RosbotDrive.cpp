#include "RosbotDrive.h"
#include "RosbotRegulatorCMSIS.h"

#define FOR(x) for(int i=0;i<x;i++)

static const DRV8848_Params_t DEFAULT_MDRV1_PARAMS{
    MOT1A_IN,
    MOT1B_IN,
    MOT1_PWM,
    MOT2A_IN,
    MOT2B_IN,
    MOT2_PWM,
    MOT12_FAULT,
    MOT12_SLEEP};

static const DRV8848_Params_t DEFAULT_MDRV2_PARAMS{
    MOT3A_IN,
    MOT3B_IN,
    MOT3_PWM,
    MOT4A_IN,
    MOT4B_IN,
    MOT4_PWM,
    MOT34_FAULT,
    MOT34_SLEEP};

const RosbotWheel RosbotDrive::DEFAULT_WHEEL_PARAMS = {
    .radius = 0.0425,
    .diameter_modificator = 1.0f,
    .tyre_deflection = 1.0f,
    .gear_ratio = 46.85,
    .encoder_cpr = 48,
    .polarity = 0b00111100,
};

const RosbotRegulator_params RosbotDrive::DEFAULT_REGULATOR_PARAMS = {
    .kp = 0.8,
    .ki = 0.2,
    .kd = 0.015,
    .out_min = -1.0,
    .out_max = 1.0,
    .a_max = 6.0,
    .da_max = 8.0,
    .speed_max = 1.5,
    .dt_ms = 10};

static TIM_TypeDef * encoder_timer[] = { ENCODER_1, ENCODER_2, ENCODER_3, ENCODER_4 };
RosbotDrive * RosbotDrive::_instance = NULL;

Thread regulator_thread(osPriorityHigh);

RosbotDrive::RosbotDrive()
: _state(UNINIT)
, _regulator_output_enabled(false)
, _regulator_loop_enabled(false)
, _tspeed_mps{0,0,0,0}
, _cspeed_mps{0,0,0,0}
, _cdistance{0,0,0,0}
{}

RosbotDrive & RosbotDrive::getInstance()
{
    if(_instance==NULL)
    {
        static RosbotDrive instance;
        _instance = &instance;
    }
    return *_instance;
}

void RosbotDrive::init(const RosbotWheel & wheel_params, const RosbotRegulator_params & params)
{
    static bool initialized = false;
    if(initialized)
        return;
    
    rosbot_drive_mutex.lock();
    
    _mot_driver[0]= new DRV8848(&DEFAULT_MDRV1_PARAMS);
    _mot_driver[1]= new DRV8848(&DEFAULT_MDRV2_PARAMS);
    FOR(2) _mot[i]=_mot_driver[0]->getDCMotor((MotNum)i);
    FOR(2) _mot[i+2]=_mot_driver[1]->getDCMotor((MotNum)i);
    FOR(4) _encoder[i] = new Encoder(encoder_timer[i]);
    FOR(4) _regulator[i] = new RosbotRegulatorCMSIS(params);
    _pid_interval_s = params.dt_ms/1000.0f;
    updateWheelParams(&params->wheel_params);

    FOR(4)
    {
        _mot[i]->setPolarity(_polarity>>i & 1);
        _mot[i]->init(PWM_DEFAULT_FREQ_HZ);
        _mot[i]->setDriveMode(true);
        _encoder[i]->setPolarity(_polarity>>(i+4) & 1);
        _encoder[i]->init();
        _pid_instance[i]->Kp = _pid_params.kp;
        _pid_instance[i]->Ki = _pid_params.ki;
        _pid_instance[i]->Kd = _pid_params.kd;
        arm_pid_init_f32(_pid_instance[i],1);
    }
    
    _mot_driver[0]->enable(true);
    if(ROSBOT_DRIVE_TYPE==4)
        _mot_driver[1]->enable(true);

    regulator_thread.start(callback(this,&RosbotDrive::regulatorLoop));
    initialized = true;
    _state=HALT;

    rosbot_drive_mutex.unlock();
}

void RosbotDrive::enable(bool en)
{
    if(_state == UNINIT)
        return;
    
    _mot_driver[0]->enable(en);
    if(ROSBOT_DRIVE_TYPE==4)
        _mot_driver[1]->enable(en);

    switch(_state)
    {
        case HALT:
            if(en)
                _state=OPERATIONAL;
            else
                _state=IDLE;
            break;
        case IDLE:
            if(en) _state=OPERATIONAL;
            break;
        case OPERATIONAL:
            if(!en)
            {
                _state=IDLE;
                FOR(ROSBOT_DRIVE_TYPE) 
                {
                    _mot[i]->setPower(0);
                    _tspeed_mps[i]=0;
                    arm_pid_reset_f32(_pid_instance[i]);
                }
            } 
            break;
        default:
            break;
    }
}

void RosbotDrive::regulatorLoop()
{
    uint64_t sleepTime;
    int32_t distance;
    float pidout, a, b, c, d, tmp_tspeed = 0.0;
    while (1)
    {
        sleepTime = Kernel::get_ms_count() + _pid_params.dt_ms;
        if (_regulator_loop_enabled) //TODO: change to mutex with fixed held time
        {

            a = _wheel_coefficient1 / _pid_interval_s;
            // b = _pid_params.a_max * _pid_interval_s;
            // c = _pid_params.da_max * _pid_interval_s;
            FOR(ROSBOT_DRIVE_TYPE)
            {
                distance = _encoder[i]->getCount();
                _cspeed_mps[i] = (float)(distance - _cdistance[i]) * a;
                _cdistance[i] = distance;

                d = _tspeed_mps[i] - _cspeed_mps[i];
                tmp_tspeed = _tspeed_mps[i];

                // if(isacc((float*)&_tspeed_mps[i],(float*)&_cspeed_mps[i])) // acceleration
                // {
                //     if(fabs(d) > b)
                //         tmp_tspeed += sgn(tmp_tspeed)*b;
                //     else
                //         tmp_tspeed = _tspeed_mps[i];
                // }
                // else // deacceleration
                // {
                //     if(fabs(d) > c)
                //         tmp_tspeed -= sgn(tmp_tspeed)*c;
                //     else
                //         tmp_tspeed = _tspeed_mps[i];
                // }

                if (tmp_tspeed > _pid_params.speed_max)
                    tmp_tspeed = _pid_params.speed_max;
                else if (tmp_tspeed < -_pid_params.speed_max)
                    tmp_tspeed = -_pid_params.speed_max;

                _error[i] = tmp_tspeed - _cspeed_mps[i];
            }
            //TODO: setup sequence
            if ((_state == OPERATIONAL) && _regulator_output_enabled)
            {
                FOR(ROSBOT_DRIVE_TYPE)
                {
                    pidout = arm_pid_f32(_pid_instance[i], _error[i]);
                    _pidout[i] = (pidout > _pid_params.out_max ? _pid_params.out_max : (pidout < _pid_params.out_min ? _pid_params.out_min : pidout));
                    _mot[i]->setPower(_pidout[i]);
                }
            }
        }
        ThisThread::sleep_until(sleepTime);
    }
}

void RosbotDrive::updateTargetSpeed(const NewTargetSpeed_t * new_speed)
{
    if(_state != OPERATIONAL)
        return;
    switch(new_speed->mode)
    {
        case DUTY_CYCLE:
            if(!_regulator_output_enabled)
                FOR(ROSBOT_DRIVE_TYPE) {_mot[i]->setPower(new_speed->speed[i]);}
            break;
        case MPS:
            if(_regulator_output_enabled)
                FOR(ROSBOT_DRIVE_TYPE) {_tspeed_mps[i]=new_speed->speed[i];}
            break;
        default:
            return;
    }
}

float RosbotDrive::getDistance(RosbotMotNum mot_num)
{
    return (float)_wheel_coefficient1 * _cdistance[mot_num];
}

float RosbotDrive::getAngularPos(RosbotMotNum mot_num)
{
    return (float)_wheel_coefficient2 * _cdistance[mot_num];
}

int32_t RosbotDrive::getEncoderTicks(RosbotMotNum mot_num)
{
    return _cdistance[mot_num];
}

float RosbotDrive::getSpeed(RosbotMotNum mot_num)
{
    return _cspeed_mps[mot_num];
}

void RosbotDrive::updateWheelCoefficients(const RosbotWheel & params)
{
    _wheel_coefficient1 =  2 * M_PI * params.radius / (params.gear_ratio * params.encoder_cpr * params.tyre_deflection);
    _wheel_coefficient2 =  2 * M_PI / (params.gear_ratio * params.encoder_cpr);
}

void RosbotDrive::updatePidParams(const RosbotRegulator_params_t * params, bool reset)
{
    _regulator_loop_enabled = false;
    _pid_params = *params;
    FOR(ROSBOT_DRIVE_TYPE)
    {
        _pid_instance[i]->Kp = _pid_params.kp;
        _pid_instance[i]->Ki = _pid_params.ki;
        _pid_instance[i]->Kd = _pid_params.kd;
        arm_pid_init_f32(_pid_instance[i],reset);
    }
    _pid_interval_s = params->dt_ms/1000.0f;
    _regulator_loop_enabled = true;
}

void RosbotDrive::stop()
{
    switch(_state)
    {
        case IDLE:
            _state=HALT;
            _mot_driver[0]->enable(true);
            if(ROSBOT_DRIVE_TYPE==4)
                _mot_driver[1]->enable(true);
            break;
        case OPERATIONAL:
            _state=HALT;
            FOR(ROSBOT_DRIVE_TYPE) 
            {
                _mot[i]->setPower(0);
                _tspeed_mps[i]=0;
                arm_pid_reset_f32(_pid_instance[i]);
            }
            break;
        default:
            break;
    }
}

void RosbotDrive::enablePidReg(bool en)
{
    _regulator_output_enabled = en;
}

bool RosbotDrive::isPidEnabled() 
{
    return _regulator_output_enabled;
}

void RosbotDrive::getPidDebugData(PidDebugData_t * data, RosbotMotNum mot_num)
{
    // CriticalSectionLock lock;
    data->cspeed=_cspeed_mps[mot_num];
    data->tspeed=_tspeed_mps[mot_num];
    data->error=_error[mot_num];
    data->pidout=_pidout[mot_num];
}

float RosbotDrive::getSpeed(RosbotMotNum mot_num, SpeedMode mode)
{
    switch(mode)
    {
        case TICSKPS:
            return _cspeed_mps[mot_num]/_wheel_coefficient1;
        case MPS:
            return _cspeed_mps[mot_num];
        case DUTY_CYCLE:
            if(_mot[mot_num]!=NULL)
                return _mot[mot_num]->getDutyCycle();
        default:
            return 0.0;
    }
}

void RosbotDrive::resetDistance()
{
    _regulator_loop_enabled = false;
    FOR(ROSBOT_DRIVE_TYPE)
    {
        _mot[i]->setPower(0);
        _encoder[i]->resetCount();
        arm_pid_reset_f32(_pid_instance[i]);
        _tspeed_mps[i]=0;
        _cspeed_mps[i]=0;
        _cdistance[i]=0;
    }
    _regulator_loop_enabled = true;
}