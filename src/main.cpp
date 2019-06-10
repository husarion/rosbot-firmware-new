#include <mbed.h>
#include <RosbotDrive.h>
#include <ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <rosbot_kinematics_layer.h>

#define JOINT_STATE_RVIZ 0

sensor_msgs::BatteryState battery_state;
geometry_msgs::Twist current_vel;
sensor_msgs::JointState joint_states;
geometry_msgs::TransformStamped odomTransform;
tf::TransformBroadcaster broadcaster;

ros::NodeHandle nh;
ros::Publisher *vel_pub;
ros::Publisher *joint_state_pub;
ros::Publisher *battery_pub;

RosbotOdometry_t odometry;
RosbotDrive * driver;

// the arrays for the JointState message
char *name[] = {"front_left_wheel_hinge", "front_right_wheel_hinge", "rear_left_wheel_hinge", "rear_right_wheel_hinge"};
double pos[] = {0, 0, 0, 0};
double vel[] = {0, 0, 0, 0};
double eff[] = {0, 0, 0, 0};

void initBatteryPublisher()
{
    battery_state.power_supply_status = battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state.power_supply_health = battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state.power_supply_technology = battery_state.POWER_SUPPLY_TECHNOLOGY_LION;
    battery_pub = new ros::Publisher("/battery", &battery_state);
    nh.advertise(*battery_pub);
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

static void initTfBroadcaster()
{
    broadcaster.init(nh);
    odomTransform.header.frame_id = "/odom";
    odomTransform.child_frame_id = "/base_link";
    odomTransform.transform.translation.x = 0.0;
    odomTransform.transform.translation.y = 0.0;
    odomTransform.transform.translation.z = 0.0;
    odomTransform.transform.rotation.x = 0.0;
    odomTransform.transform.rotation.y = 0.0;
    odomTransform.transform.rotation.z = 0.0;
    odomTransform.transform.rotation.w = 1.0;
}

void initJointStatePublisher()
{
    joint_state_pub = new ros::Publisher("/joint_states", &joint_states);
    nh.advertise(*joint_state_pub);

    joint_states.header.frame_id = "base_link";

    //assigning the arrays to the message
    joint_states.name = name;
    joint_states.position = pos;
    joint_states.velocity = vel;
    joint_states.effort = eff;

    //setting the length
    joint_states.name_length = 4;
    joint_states.position_length = 4;
    joint_states.velocity_length = 4;
    joint_states.effort_length = 4;
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

    odomTransform.transform.translation.x = odometry.robot_x_pos;
    odomTransform.transform.translation.y = odometry.robot_y_pos;
    odomTransform.transform.rotation = tf::createQuaternionFromYaw(odometry.robot_angular_pos);
}

int main()
{
    driver = RosbotDrive::getInstance(&ROSBOT_PARAMS);
    driver->init();
    driver->enable(true);
    driver->enablePidReg(true);

    nh.initNode();

    ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &velocityCallback);
    nh.subscribe(cmd_vel_sub);
    
    initBatteryPublisher();
    initTfBroadcaster();
    initVelocityPublisher();
    
#if JOINT_STATE_RVIZ
    initJointStatePublisher();
#endif

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
            
            odomTransform.header.stamp = nh.now();
            broadcaster.sendTransform(odomTransform);
            vel_pub->publish(&current_vel);

#if JOINT_STATE_RVIZ
            pos[0] = odometry.wheel_FL_ang_pos;
            pos[1] = odometry.wheel_FR_ang_pos;
            pos[2] = odometry.wheel_RL_ang_pos;
            pos[3] = odometry.wheel_RR_ang_pos;
            joint_states.position = pos;
            joint_states.header.stamp = odomTransform.header.stamp; 
            joint_state_pub->publish(&joint_states);
#endif
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