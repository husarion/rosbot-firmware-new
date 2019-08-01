#include "RosbotDrive.h"
#define FOR(x) for(int i=0;i<x;i++)

/***************************CMSIS-DSP-PID***************************/
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#if 0
/**
* @brief  Process function for the floating-point PID Control.
* @param[in,out] S   is an instance of the floating-point PID Control structure
* @param[in]     in  input sample to process
* @return out processed output sample.
*/
static __INLINE float32_t arm_pid_f32(
    arm_pid_instance_f32 *S,
    float32_t in)
{
    float32_t out;

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
    out = (S->A0 * in) +
          (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

    /* Update state */
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

    /* return to application */
    return (out);
}
#endif

void arm_pid_init_f32(
  arm_pid_instance_f32 * S,
  int32_t resetStateFlag)
{

  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;

  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((float32_t) 2.0 * S->Kd);

  /* Derived coefficient A2 */
  S->A2 = S->Kd;

  /* Check whether state needs reset or not */
  if(resetStateFlag)
  {
    /* Clear the state buffer.  The size will be always 3 samples */
    memset(S->state, 0, 3u * sizeof(float32_t));
  }

}

/**    
* @brief  Reset function for the floating-point PID Control.   
* @param[in] *S	Instance pointer of PID control data structure.   
* @return none.    
* \par Description:   
* The function resets the state buffer to zeros.    
*/
void arm_pid_reset_f32(
  arm_pid_instance_f32 * S)
{

  /* Clear the state buffer.  The size will be always 3 samples */
  memset(S->state, 0, 3u * sizeof(float32_t));
}
/***************************CMSIS-DSP-PID***************************/

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

const RosbotWheel_t RosbotDrive::DEFAULT_WHEEL_PARAMS = {
    .radius = 0.0425,
    .diameter_modificator = 1.0f,
    .tyre_deflection = 1.0f,
    .gear_ratio = 46.85,
    .encoder_cpr = 48
};

const RosbotDrivePid_t RosbotDrive::DEFAULT_PID_PARAMS = {
    .kp = 0.8,
    .ki = 0.2,
    .kd = 0.015,
    .out_min = -1.0,
    .out_max = 1.0,
    .a_max = 6.0,
    .da_max = 8.0,
    .speed_max = 1.5,
    .dt_ms = 10};

#if ROSBOT_DRIVE_TYPE == 4
static DRV8848 mot_driver1(&DEFAULT_MDRV1_PARAMS); 
static DRV8848 mot_driver2(&DEFAULT_MDRV2_PARAMS);
static Encoder encoder[ROSBOT_DRIVE_TYPE] = 
{
    Encoder(ENCODER_1),
    Encoder(ENCODER_2),
    Encoder(ENCODER_3),
    Encoder(ENCODER_4)
};
#else
static DRV8848 mot_driver1(&DEFAULT_MDRV1_PARAMS);
static Encoder encoder[ROSBOT_DRIVE_TYPE] = 
{
    Encoder(ENCODER_1),
    Encoder(ENCODER_2)
};
#endif

static arm_pid_instance_f32 pid_instace[4];
RosbotDrive * RosbotDrive::_instance = NULL;

Thread regulator_thread(osPriorityHigh);

int RosbotDrive::getRosbotDriveType()
{return ROSBOT_DRIVE_TYPE;}

RosbotDrive::RosbotDrive(const RosbotDrive_params_t * params)
: _state(UNINIT)
, _pid_state(false)
, _regulator_state(true)
, _pid_params(params->pid_params)
, _polarity(params->polarity)
{
#if ROSBOT_DRIVE_TYPE == 2
    _mot_driver[0] = &mot_driver1;
    FOR(ROSBOT_DRIVE_TYPE) _encoder[i] = &encoder[i]; 
#elif ROSBOT_DRIVE_TYPE == 4
    _mot_driver[0] = &mot_driver1;
    _mot_driver[1] = &mot_driver2;
    FOR(ROSBOT_DRIVE_TYPE) _encoder[i] = &encoder[i]; 
#endif
    FOR(4)
    {
        _mot[i] = NULL;
        _pid_instance[i] = &pid_instace[i];
        _tspeed_mps[i]=0;
        _cspeed_mps[i]=0;
        _cdistance[i]=0;
    }
    updateWheelParams(&params->wheel_params);
    _pid_interval_s = params->pid_params.dt_ms/1000.0f;
}

RosbotDrive * RosbotDrive::getInstance(const RosbotDrive_params_t * params)
{
    if(_instance==NULL && params!=NULL)
    {
        _instance = new RosbotDrive(params);
    }
    return _instance;
}

void RosbotDrive::init(void)
{
    static bool initialized = false;
    if(initialized)
        return;
    
    switch(ROSBOT_DRIVE_TYPE)
    {
        case 2:
            FOR(2) _mot[i]=_mot_driver[0]->getDCMotor((MotNum)i);
            break;
        case 4:
            FOR(2) _mot[i]=_mot_driver[0]->getDCMotor((MotNum)i);
            FOR(2) _mot[i+2]=_mot_driver[1]->getDCMotor((MotNum)i);
            break;
        default:
            return;
    }
    
    FOR(ROSBOT_DRIVE_TYPE)
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

#if 0 // old implementation
void RosbotDrive::regulatorLoop()
{
    //TODO: Add acceleration and deacceleration cotrol and fault handling
    uint64_t sleepTime;
    int32_t distance;
    float pidout,tmp, cspeed_mps;
    while (1)
    {
        sleepTime = Kernel::get_ms_count() + _pid_params.dt_ms;
        tmp = _wheel_coefficient1 * 1000.0f / _pid_params.dt_ms; // TODO: update with wheel_coefficient
        FOR(ROSBOT_DRIVE_TYPE)
        {
            distance = _encoder[i]->getCount();
            cspeed_mps = (float)(distance - _cdistance[i])*tmp;
            _caccel_mps2[i] = (cspeed_mps - _cspeed_mps[i])/(1000.0f / _pid_params.dt_ms);
            _cspeed_mps[i] = cspeed_mps; 
            _cdistance[i] = distance;
        }
        if((_state == OPERATIONAL) && _pid_state)
        {
            FOR(ROSBOT_DRIVE_TYPE)
            {
                _error[i] = _tspeed_mps[i]-_cspeed_mps[i];
                pidout = arm_pid_f32(_pid_instance[i],_error[i]);
                _pidout[i] = (pidout > _pid_params.out_max ? _pid_params.out_max :(pidout < _pid_params.out_min ? _pid_params.out_min : pidout));
                _mot[i]->setPower(_pidout[i]);  
            } 
        }
        ThisThread::sleep_until(sleepTime);
    }
}
#endif

template <typename T> int sgn(T val) {
    return (T(0) > val) ? -1 : 1 ;
}

static inline bool isacc(float * tspeed, float * cspeed)
{
    if (sgn(*tspeed) == sgn(*cspeed))
        return (fabs(*tspeed)>fabs(*cspeed));
    else
        return false;
}

void RosbotDrive::regulatorLoop()
{
    uint64_t sleepTime;
    int32_t distance;
    float pidout, a, b, c, d, tmp_tspeed = 0.0;
    while (1)
    {
        if (_regulator_state)
        {

            sleepTime = Kernel::get_ms_count() + _pid_params.dt_ms;
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
            if ((_state == OPERATIONAL) && _pid_state)
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
            if(!_pid_state)
                FOR(ROSBOT_DRIVE_TYPE) {_mot[i]->setPower(new_speed->speed[i]);}
            break;
        case MPS:
            if(_pid_state)
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

void RosbotDrive::updateWheelParams(const RosbotWheel_t * params)
{
    _wheel_coefficient1 =  2 * M_PI * params->radius / (params->gear_ratio * params->encoder_cpr * params->tyre_deflection);
    _wheel_coefficient2 =  2 * M_PI / (params->gear_ratio * params->encoder_cpr);
}

void RosbotDrive::updatePidParams(const RosbotDrivePid_t * params, bool reset)
{
    _regulator_state = false;
    _pid_params = *params;
    FOR(ROSBOT_DRIVE_TYPE)
    {
        _pid_instance[i]->Kp = _pid_params.kp;
        _pid_instance[i]->Ki = _pid_params.ki;
        _pid_instance[i]->Kd = _pid_params.kd;
        arm_pid_init_f32(_pid_instance[i],reset);
    }
    _pid_interval_s = params->dt_ms/1000.0f;
    _regulator_state = true;
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
    _pid_state = en;
}

bool RosbotDrive::isPidEnabled() 
{
    return _pid_state;
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
    _regulator_state = false;
    FOR(ROSBOT_DRIVE_TYPE)
    {
        _mot[i]->setPower(0);
        _encoder[i]->resetCount();
        arm_pid_reset_f32(_pid_instance[i]);
        _tspeed_mps[i]=0;
        _cspeed_mps[i]=0;
        _cdistance[i]=0;
    }
    _regulator_state = true;
}