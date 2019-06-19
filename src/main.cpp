/** @file main.cpp
 * ROSbot firmware - 19th of June 2019 
 */
#include <mbed.h>
#include <RosbotDrive.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <rosbot_kinematics.h>
#include <tf/tf.h>
#include <rosbot_sensors.h>

geometry_msgs::Twist current_vel;
sensor_msgs::JointState joint_states;
sensor_msgs::BatteryState battery_state;
sensor_msgs::Range range_msg[4];
geometry_msgs::PoseStamped pose;

ros::NodeHandle nh;
ros::Publisher *vel_pub;
ros::Publisher *joint_state_pub;
ros::Publisher *battery_pub;
ros::Publisher *range_pub[4];
ros::Publisher *pose_pub;

RosbotOdometry_t odometry;
RosbotDrive * driver;
MultiDistanceSensor * distance_sensors;

// JointState
char * joint_state_name[] = {"front_left_wheel_hinge", "front_right_wheel_hinge", "rear_left_wheel_hinge", "rear_right_wheel_hinge"};
double pos[] = {0, 0, 0, 0};
double vel[] = {0, 0, 0, 0};
double eff[] = {0, 0, 0, 0};

// Range
const char * range_id[] = {"range_fr","range_fl","range_rr","range_rl"};
const char * range_pub_names[] = {"/range/fr","/range/fl","/range/rr","/range/rl"};

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
    pose.header.frame_id = "rosbot";
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
    vel_pub = new ros::Publisher("velocity", &current_vel);
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
    setRosbotSpeed(driver,twist_msg.linear.x, twist_msg.angular.z);
}

static void updateOdometryAndSpeed(float dtime)
{
    updateRosbotOdometry(driver,&odometry,dtime);
    current_vel.linear.x = sqrt(odometry.robot_x_vel * odometry.robot_x_vel + odometry.robot_y_vel * odometry.robot_y_vel);
    current_vel.angular.z = odometry.robot_angular_vel;

    pose.pose.position.x = odometry.robot_x_pos;
    pose.pose.position.y = odometry.robot_y_pos;
    pose.pose.orientation = tf::createQuaternionFromYaw(odometry.robot_angular_pos);
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

int main()
{
    DigitalOut sens_power(SENS_POWER_ON,1);
    driver = RosbotDrive::getInstance(&ROSBOT_PARAMS);
    distance_sensors = MultiDistanceSensor::getInstance(&SENSORS_PIN_DEF);

    if(distance_sensors->init()!=4)
        nh.logerror("VL53L0X sensros initialisation failure!");
    driver->init();
    driver->enable(true);
    driver->enablePidReg(true);

    events::EventQueue * q = mbed_event_queue();
    initBatteryWatchdog(q,5,8.0);

    nh.initNode();
    ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &velocityCallback);
    nh.subscribe(cmd_vel_sub);
    
    initBatteryPublisher();
    initPosePublisher();
    initVelocityPublisher();
    initRangePublisher();
    initJointStatePublisher();

#if defined(MEMORY_DEBUG_INFO)
    print_debug_info();
#endif /* MEMORY_DEBUG_INFO */ 

    for(int i=0;i<4;i++)
    {
        VL53L0X * sensor = distance_sensors->getSensor(i);
        sensor->setTimeout(50); 
        sensor->startContinuous();
    }
    int spin_result;
    uint32_t spin_count=1;
    uint64_t time_old = Kernel::get_ms_count();

    while (1)
    {
        if (spin_count % 5 == 0) /// cmd_vel, odometry, joint_states
        {
            uint64_t time_new = Kernel::get_ms_count();
            updateOdometryAndSpeed((time_new-time_old)/1000.0f);
            time_old = time_new;
            
            pose.header.stamp = nh.now();
            pose_pub->publish(&pose);
            vel_pub->publish(&current_vel);

            pos[0] = odometry.wheel_FL_ang_pos;
            pos[1] = odometry.wheel_FR_ang_pos;
            pos[2] = odometry.wheel_RL_ang_pos;
            pos[3] = odometry.wheel_RR_ang_pos;

            joint_states.position = pos;
            joint_states.header.stamp = pose.header.stamp; 
            joint_state_pub->publish(&joint_states);
        }

        if(spin_count % 50 == 0)
        {
            battery_state.voltage = readVoltage();
            battery_pub->publish(&battery_state);
        }

        if(spin_count % 20 == 0) // ~ 5 HZ
        {
            uint16_t range;
            ros::Time t = nh.now();
            for(int i=0;i<4;i++)
            {
                range = distance_sensors->getSensor(i)->readRangeContinuousMillimeters(false);
                range_msg[i].header.stamp = t;
                range_msg[i].range = (range != 65535) ? (float)range/1000.0f : -1.0f;
                range_msg[i].header.frame_id = range_id[i];
                range_pub[i]->publish(&range_msg[i]);
            }
        }
        
        spin_result = nh.spinOnce();
        if(spin_result != ros::SPIN_OK)
        {
            nh.logwarn(spin_result == -1 ? "SPIN_ERR" : "SPIN_TIMEOUT");
        }
        spin_count++;
        ThisThread::sleep_for(10);
    }
}