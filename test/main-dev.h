/** @file main.cpp
 * ROSbot firmware - 27th of June 2019 
 */
#include <mbed.h>
#include <RosbotDrive.h>
#include <rosbot_kinematics.h>
#include <rosbot_sensors.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosbot/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <tf/tf.h>
#include <std_msgs/UInt8.h>
#include <rosbot/Configuration.h>
#include <map>
#include <string>

#define MAIN_LOOP_INTERVAL_MS 10

geometry_msgs::Twist current_vel;
sensor_msgs::JointState joint_states;
sensor_msgs::BatteryState battery_state;
sensor_msgs::Range range_msg[4];
geometry_msgs::PoseStamped pose;
std_msgs::UInt8 button_msg;
rosbot::Imu imu_msg;
ros::NodeHandle nh;
ros::Publisher *vel_pub;
ros::Publisher *joint_state_pub;
ros::Publisher *battery_pub;
ros::Publisher *range_pub[4];
ros::Publisher *pose_pub;
ros::Publisher *button_pub;
ros::Publisher *imu_pub;

rosbot_kinematics::RosbotOdometry_t odometry;
RosbotDrive * driver;
MultiDistanceSensor * distance_sensors;
volatile bool distance_sensors_enabled = false;
volatile bool joint_states_enabled = false;

DigitalOut led2(LED2,0);
DigitalOut led3(LED3,0);
InterruptIn button1(BUTTON1);
InterruptIn button2(BUTTON2);
volatile bool button1_publish_flag = false;
volatile bool button2_publish_flag = false;

volatile bool is_speed_watchdog_enabled = true;
volatile bool is_speed_watchdog_active = false;
int speed_watchdog_interval = 1000; //ms
volatile uint64_t last_speed_command_time=0;

static void button1Callback()
{
    button1_publish_flag = true;
}

static void button2Callback()
{
    button2_publish_flag = true;
}

// JointState
char * joint_state_name[] = {"front_left_wheel_hinge", "front_right_wheel_hinge", "rear_left_wheel_hinge", "rear_right_wheel_hinge"};
double pos[] = {0, 0, 0, 0};
double vel[] = {0, 0, 0, 0};
double eff[] = {0, 0, 0, 0};

// Range
const char * range_id[] = {"range_fr","range_fl","range_rr","range_rl"};
const char * range_pub_names[] = {"/range/fr","/range/fl","/range/rr","/range/rl"};

static void initImuPublisher()
{
    imu_pub = new ros::Publisher("/mpu9250", &imu_msg);
    nh.advertise(*imu_pub);
}

static void initButtonPublisher()
{
    button_pub = new ros::Publisher("/buttons", &button_msg);
    nh.advertise(*button_pub);
}

static void initRangePublisher()
{
    for(int i=0;i<4;i++)
    {
        range_msg[i].field_of_view = 0.26;
        range_msg[i].min_range = 0.03;
        range_msg[i].max_range = 0.90;
        range_msg[i].radiation_type = sensor_msgs::Range::INFRARED;
        range_pub[i] = new ros::Publisher(range_pub_names[i],&range_msg[i]);
        nh.advertise(*range_pub[i]);
    }
}

static void initBatteryPublisher()
{
    battery_state.power_supply_status = battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state.power_supply_health = battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state.power_supply_technology = battery_state.POWER_SUPPLY_TECHNOLOGY_LION;
    battery_pub = new ros::Publisher("/battery", &battery_state);
    nh.advertise(*battery_pub);
}

static void initPosePublisher()
{
    pose.header.frame_id = "odom";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    pose_pub = new ros::Publisher("/pose", &pose);
    nh.advertise(*pose_pub);
}

static void initVelocityPublisher()
{
    current_vel.linear.x = 0;
    current_vel.linear.y = 0;
    current_vel.linear.z = 0;
    current_vel.angular.x = 0;
    current_vel.angular.y = 0;
    current_vel.angular.z = 0;
    vel_pub = new ros::Publisher("/velocity", &current_vel);
    nh.advertise(*vel_pub);
}

static void initJointStatePublisher()
{
    joint_state_pub = new ros::Publisher("/joint_states", &joint_states);
    nh.advertise(*joint_state_pub);

    joint_states.header.frame_id = "base_link";

    //assigning the arrays to the message
    joint_states.name = joint_state_name;
    joint_states.position = pos;
    // joint_states.velocity = vel;
    // joint_states.effort = eff;

    //setting the length
    joint_states.name_length = 4;
    joint_states.position_length = 4;
    // joint_states.velocity_length = 4;
    // joint_states.effort_length = 4;
}

static void velocityCallback(const geometry_msgs::Twist &twist_msg)
{
    rosbot_kinematics::setRosbotSpeed(driver,twist_msg.linear.x, twist_msg.angular.z);
    last_speed_command_time = rtos::Kernel::get_ms_count();
    is_speed_watchdog_active = false;
}

static void updateOdometryAndSpeed(float dtime)
{
    rosbot_kinematics::updateRosbotOdometry(driver,&odometry,dtime);
    current_vel.linear.x = sqrt(odometry.robot_x_vel * odometry.robot_x_vel + odometry.robot_y_vel * odometry.robot_y_vel);
    current_vel.angular.z = odometry.robot_angular_vel;

    pose.pose.position.x = odometry.robot_x_pos;
    pose.pose.position.y = odometry.robot_y_pos;
    pose.pose.orientation = tf::createQuaternionFromYaw(odometry.robot_angular_pos);
}

class ConfigFunctionality
{
public:
    typedef uint8_t (ConfigFunctionality::*configuration_srv_fun_t)(const char *datain, const char **dataout);
    static ConfigFunctionality *getInstance();
    configuration_srv_fun_t findFunctionality(const char *command);
    uint8_t setLed(const char *datain, const char **dataout);
    uint8_t enableImu(const char *datain, const char **dataout);
    uint8_t enableDistanceSensors(const char *datain, const char **dataout);
    uint8_t enableJointStates(const char *datain, const char **dataout);
    uint8_t resetOdom(const char *datain, const char **dataout);
    uint8_t enableSpeedWatchdog(const char *datain, const char **dataout);
    uint8_t getAngle(const char *datain, const char **dataout);
    uint8_t resetImu(const char *datain, const char **dataout);
    uint8_t setMotorsAccelDeaccel(const char *datain, const char **dataout);

private:
    ConfigFunctionality();
    char _buffer[50];
    static ConfigFunctionality *_instance;
    static const char SLED_COMMAND[];
    static const char EIMU_COMMAND[];
    static const char EDSE_COMMAND[];
    static const char EJSM_COMMAND[];
    static const char RODOM_COMMAND[];
    static const char EWCH_COMMAND[];
    static const char RIMU_COMMAND[];
    static const char SMAD_COMMAND[];
    map<std::string, configuration_srv_fun_t> _commands;
};

ConfigFunctionality * ConfigFunctionality::_instance=NULL;

const char ConfigFunctionality::SLED_COMMAND[]="SLED";
const char ConfigFunctionality::EIMU_COMMAND[]="EIMU";
const char ConfigFunctionality::EDSE_COMMAND[]="EDSE";
const char ConfigFunctionality::EJSM_COMMAND[]="EJSM";
const char ConfigFunctionality::RODOM_COMMAND[]="RODOM";
const char ConfigFunctionality::EWCH_COMMAND[]="EWCH";
const char ConfigFunctionality::RIMU_COMMAND[]="RIMU";
const char ConfigFunctionality::SMAD_COMMAND[]="SMAD";

ConfigFunctionality::ConfigFunctionality()
{
    _commands[SLED_COMMAND] = &ConfigFunctionality::setLed;
    _commands[EIMU_COMMAND] = &ConfigFunctionality::enableImu;
    _commands[EDSE_COMMAND] = &ConfigFunctionality::enableDistanceSensors;
    _commands[EJSM_COMMAND] = &ConfigFunctionality::enableJointStates;
    _commands[RODOM_COMMAND] = &ConfigFunctionality::resetOdom;
    _commands[EWCH_COMMAND] = &ConfigFunctionality::enableSpeedWatchdog;
    _commands[RIMU_COMMAND] = &ConfigFunctionality::resetImu;
}

ConfigFunctionality::configuration_srv_fun_t ConfigFunctionality::findFunctionality(const char *command)
{
    std::map<std::string, configuration_srv_fun_t>::iterator it = _commands.find(command);
    if(it != _commands.end())
        return it->second;
    else
        return NULL;
}

uint8_t ConfigFunctionality::resetImu(const char *datain, const char **dataout)
{
    *dataout = NULL;
    rosbot_sensors::resetImu();
    return rosbot::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::setMotorsAccelDeaccel(const char *datain, const char **dataout)
{
    float accel, deaccel;
    *dataout = NULL;
    //TODO: zakres
    if(sscanf(datain,"%f %f",&accel, deaccel) == 2)
    {
        RosobtDrivePid new_pid_params = RosbotDrive::DEFAULT_REGULATOR_PARAMS;
        new_pid_params.a_max = accel;
        new_pid_params.da_max = deaccel;
        driver->updatePidParams(&new_pid_params,true);
        return rosbot::Configuration::Response::SUCCESS; 
    }
    return rosbot::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::resetOdom(const char *datain, const char **dataout)
{
    *dataout = NULL;
    rosbot_kinematics::resetRosbotOdometry(driver,&odometry);
    return rosbot::Configuration::Response::SUCCESS;
}

uint8_t ConfigFunctionality::enableImu(const char *datain, const char **dataout)
{
    int en;
    *dataout = NULL;
    if(sscanf(datain,"%d",&en) == 1)
    {
        events::EventQueue * q = mbed_event_queue();
        q->call(Callback<void(int)>(&rosbot_sensors::enableImu),en);
        return rosbot::Configuration::Response::SUCCESS; 
    }
    return rosbot::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableJointStates(const char *datain, const char **dataout)
{
    int en;
    *dataout = NULL;
    if(sscanf(datain,"%d",&en) == 1)
    {
        joint_states_enabled = (en == 0 ? false : true);
        return rosbot::Configuration::Response::SUCCESS; 
    }
    return rosbot::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableDistanceSensors(const char *datain, const char **dataout)
{
    int en;
    *dataout = NULL;
    if(sscanf(datain,"%d",&en) == 1)
    {
        if(en == 0)
        {
            distance_sensors_enabled = false;
            for(int i=0;i<4;i++)
            {
                VL53L0X * sensor = distance_sensors->getSensor(i);
                sensor->stopContinuous();
            }
        }
        else
        {
            distance_sensors_enabled = true;
            for(int i=0;i<4;i++)
            {
                VL53L0X * sensor = distance_sensors->getSensor(i);
                sensor->setTimeout(50); 
                sensor->startContinuous();
            }
        }
        return rosbot::Configuration::Response::SUCCESS; 
    }
    return rosbot::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::setLed(const char *datain, const char **dataout)
{
    int led_num, led_state;
    *dataout = NULL;
    if(sscanf(datain,"%d %d", &led_num, &led_state) == 2)
    {
        switch(led_num)
        {
            case 2:
                led2 = led_state;
                return rosbot::Configuration::Response::SUCCESS;
            case 3:
                led3 = led_state;
                return rosbot::Configuration::Response::SUCCESS;
            default:
                break;
        }
    }
    return rosbot::Configuration::Response::FAILURE;
}

uint8_t ConfigFunctionality::enableSpeedWatchdog(const char *datain, const char **dataout)
{
    int en;
    *dataout = NULL;
    if(sscanf(datain,"%d",&en) == 1)
    {
        is_speed_watchdog_enabled = (en == 0 ? false : true);
        return rosbot::Configuration::Response::SUCCESS; 
    }
    return rosbot::Configuration::Response::FAILURE;
}

ConfigFunctionality * ConfigFunctionality::getInstance()
{
    if(_instance == NULL)
    {
        _instance = new ConfigFunctionality();
    }
    return _instance;
}

void responseCallback(const rosbot::Configuration::Request & req, rosbot::Configuration::Response & res)
{
    ConfigFunctionality * config_functionality = ConfigFunctionality::getInstance();
    ConfigFunctionality::configuration_srv_fun_t fun = config_functionality->findFunctionality(req.command); 
    if(fun != NULL)
    {
        nh.loginfo("Command found!");
        res.result = (config_functionality->*fun)(req.data, &res.data);
    }
    else
    {
        nh.loginfo("Command not found!");
        res.result = rosbot::Configuration::Response::COMMAND_NOT_FOUND;
    }
}

#if defined(MEMORY_DEBUG_INFO)
#define MAX_THREAD_INFO 10

mbed_stats_heap_t heap_info;
mbed_stats_stack_t stack_info[ MAX_THREAD_INFO ];

int print_debug_info()
{
    debug("\nThis message is from debug function");
    debug_if(1,"\nThis message is from debug_if function");
    debug_if(0,"\nSOMETHING WRONG!!! This message from debug_if function shouldn't show on bash");
    
    printf("\nMemoryStats:");
    mbed_stats_heap_get( &heap_info );
    printf("\n\tBytes allocated currently: %d", heap_info.current_size);
    printf("\n\tMax bytes allocated at a given time: %d", heap_info.max_size);
    printf("\n\tCumulative sum of bytes ever allocated: %d", heap_info.total_size);
    printf("\n\tCurrent number of bytes allocated for the heap: %d", heap_info.reserved_size);
    printf("\n\tCurrent number of allocations: %d", heap_info.alloc_cnt);
    printf("\n\tNumber of failed allocations: %d", heap_info.alloc_fail_cnt);
    
    mbed_stats_stack_get( &stack_info[0] );
    printf("\nCumulative Stack Info:");
    printf("\n\tMaximum number of bytes used on the stack: %d", stack_info[0].max_size);
    printf("\n\tCurrent number of bytes allocated for the stack: %d", stack_info[0].reserved_size);
    printf("\n\tNumber of stacks stats accumulated in the structure: %d", stack_info[0].stack_cnt);
    
    mbed_stats_stack_get_each( stack_info, MAX_THREAD_INFO );
    printf("\nThread Stack Info:");
    for(int i=0;i < MAX_THREAD_INFO; i++) {
        if(stack_info[i].thread_id != 0) {
            printf("\n\tThread: %d", i);
            printf("\n\t\tThread Id: 0x%08X", stack_info[i].thread_id);
            printf("\n\t\tMaximum number of bytes used on the stack: %d", stack_info[i].max_size);
            printf("\n\t\tCurrent number of bytes allocated for the stack: %d", stack_info[i].reserved_size);
            printf("\n\t\tNumber of stacks stats accumulated in the structure: %d", stack_info[i].stack_cnt); 
        }        
    }
    
    printf("\nDone...\n\n");
}
#endif /* MEMORY_DEBUG_INFO */

void test()
{
    DigitalOut sens_power(SENS_POWER_ON,1);
    driver = RosbotDrive::getInstance(&rosbot_kinematics::ROSBOT_PARAMS);
    distance_sensors = MultiDistanceSensor::getInstance(&rosbot_sensors::SENSORS_PIN_DEF);

    driver->init();
    driver->enable(true);
    driver->enablePidReg(true);

    button1.mode(PullUp);
    button2.mode(PullUp);
    button1.fall(button1Callback);
    button2.fall(button2Callback);

    nh.initNode();

    if(distance_sensors->init(400000)!=4)
        nh.logerror("VL53L0X sensros initialisation failure!");

    if(rosbot_sensors::initImu()!=INV_SUCCESS)
       nh.logerror("MPU9250 initialisation failure!");

    ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &velocityCallback);
    ros::ServiceServer<rosbot::Configuration::Request,rosbot::Configuration::Response> config_srv("/config", responseCallback);
    nh.advertiseService(config_srv);
    nh.subscribe(cmd_vel_sub);
    
    initBatteryPublisher();
    initPosePublisher();
    initVelocityPublisher();
    initRangePublisher();
    initJointStatePublisher();
    initImuPublisher();
    initButtonPublisher();

    nh.spinOnce();

#if defined(MEMORY_DEBUG_INFO)
    print_debug_info();
#endif /* MEMORY_DEBUG_INFO */ 

    int spin_result;
    uint32_t spin_count=1;
    uint64_t last_spin_time=0 ,curr_time = 0;

    while (1)
    {
        curr_time = Kernel::get_ms_count(); 
        rosbot_kinematics::updateRosbotOdometry(driver,&odometry,(curr_time-last_spin_time)/1000.0f);
        last_spin_time = curr_time;
        
        if(is_speed_watchdog_enabled)
        {
            if(!is_speed_watchdog_active && (curr_time - last_speed_command_time) > speed_watchdog_interval)
            {
                rosbot_kinematics::setRosbotSpeed(driver, 0.0f, 0.0f);
                is_speed_watchdog_active = true;
            }
        }

        if(button1_publish_flag)
        {
            button1_publish_flag = false;
            if(!button1)
            {
                button_msg.data = 1;
                button_pub->publish(&button_msg);
            }
        }

        if(button2_publish_flag)
        {
            button2_publish_flag = false;
            if(!button2)
            {
                button_msg.data = 2;
                button_pub->publish(&button_msg);
            }
        }

        if (spin_count % 5 == 0) /// cmd_vel, odometry, joint_states
        {
            current_vel.linear.x = sqrt(odometry.robot_x_vel * odometry.robot_x_vel + odometry.robot_y_vel * odometry.robot_y_vel);
            current_vel.angular.z = odometry.robot_angular_vel;
            pose.pose.position.x = odometry.robot_x_pos;
            pose.pose.position.y = odometry.robot_y_pos;
            pose.pose.orientation = tf::createQuaternionFromYaw(odometry.robot_angular_pos);
            
            pose.header.stamp = nh.now();
            pose_pub->publish(&pose);
            vel_pub->publish(&current_vel);

            if(joint_states_enabled)
            {
                pos[0] = odometry.wheel_FL_ang_pos;
                pos[1] = odometry.wheel_FR_ang_pos;
                pos[2] = odometry.wheel_RL_ang_pos;
                pos[3] = odometry.wheel_RR_ang_pos;
                joint_states.position = pos;
                joint_states.header.stamp = pose.header.stamp; 
                joint_state_pub->publish(&joint_states);
            }
        }

        if(spin_count % 40 == 0)
        {
            battery_state.voltage = rosbot_sensors::updateBatteryWatchdog();
            battery_pub->publish(&battery_state);
        }

        if(spin_count % 20 == 0 && distance_sensors_enabled) // ~ 5 HZ
        {
            uint16_t range;
            ros::Time t = nh.now();
            for(int i=0;i<4;i++)
            {
                range = distance_sensors->getSensor(i)->readRangeContinuousMillimeters(false);
                range_msg[i].header.stamp = t;
                range_msg[i].range = (range != 65535) ? (float)range/1000.0f : -1.0f;
                range_pub[i]->publish(&range_msg[i]);
            }
        }
        
        osEvent evt = rosbot_sensors::imu_sensor_mail_box.get(0);

        if(evt.status == osEventMail)
        {
            rosbot_sensors::imu_meas_t * message = (rosbot_sensors::imu_meas_t*)evt.value.p;

            imu_msg.header.stamp = nh.now();
            imu_msg.orientation.x = message->orientation[0];
            imu_msg.orientation.y = message->orientation[1];
            imu_msg.orientation.z = message->orientation[2];
            imu_msg.orientation.w = message->orientation[3];
            for(int i=0;i<3;i++)
            {
                imu_msg.angular_velocity[i] = message->angular_velocity[i];
                imu_msg.linear_acceleration[i] = message->linear_velocity[i];
            }
            rosbot_sensors::imu_sensor_mail_box.free(message);
            imu_pub->publish(&imu_msg);
        }

        int spin_result = nh.spinOnce();
        if(spin_result != ros::SPIN_OK)
        {
            nh.logwarn(spin_result == -1 ? "SPIN_ERR" : "SPIN_TIMEOUT");
        }
        spin_count++;
        ThisThread::sleep_for(MAIN_LOOP_INTERVAL_MS);
    }
}