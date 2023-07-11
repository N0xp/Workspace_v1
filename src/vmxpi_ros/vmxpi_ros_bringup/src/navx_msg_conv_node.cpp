#include "TitanDriver_ros_wrapper.h"
#include "navX_ros_wrapper.h"
#include "Cobra_ros.h"
#include "Sharp_ros.h"
#include "Servo_ros.h"
#include "Ultrasonic_ros.h"
#include "IOwd_ros.h"
#include "DI_ros.h"
#include "DO_ros.h"
#include <unistd.h>

#include <std_msgs/Float32.h>
#include <vmxpi_ros/Float.h>
#include <sensor_msgs/Imu.h>
#include <cmath>

static double quatx,quaty,quatz,quatw;
static double linaccelx,linaccely,linaccelz;
static double rawgyrox,rawgyroy,rawgyroz;
static double timestamp;
static double Roll,Pitch,Yaw;
static double seq = 0;



void quatxCallback(const std_msgs::Float64::ConstPtr& msg)
{
    quatx = msg->data;

}

void quatyCallback(const std_msgs::Float64::ConstPtr& msg)
{
    quaty = msg->data;

}

void quatzCallback(const std_msgs::Float64::ConstPtr& msg)
{
    quatz = msg->data;

}

void quatwCallback(const std_msgs::Float64::ConstPtr& msg)
{
    quatw = msg->data;

}


void linaccelxCallback(const std_msgs::Float32::ConstPtr& msg)
{
    linaccelx = msg->data * 9.80;

}

void linaccelyCallback(const std_msgs::Float32::ConstPtr& msg)
{
    linaccely = msg->data* 9.80;

}

void linaccelzCallback(const std_msgs::Float32::ConstPtr& msg)
{
    linaccelz = msg->data* 9.80;

}

void rawgyroxCallback(const std_msgs::Float32::ConstPtr& msg)
{
    rawgyrox = msg->data *(M_PI / 180);

}
void rawgyroyCallback(const std_msgs::Float32::ConstPtr& msg)
{
    rawgyroy = msg->data *(M_PI / 180);

}
void rawgyrozCallback(const std_msgs::Float32::ConstPtr& msg)
{
    rawgyroz = msg->data *(M_PI / 180);

}

void RollCallback(const std_msgs::Float32::ConstPtr& msg)
{
    Roll = msg->data;

}

void PitchCallback(const std_msgs::Float32::ConstPtr& msg)
{
    Pitch = msg->data;

}

void YawCallback(const std_msgs::Float32::ConstPtr& msg)
{
    Yaw = msg->data;

}

void timestampCallback(const std_msgs::UInt64::ConstPtr& msg)
{
    timestamp = msg->data;

}


class msgsConverter
{
    private:

    public:
    ros::Subscriber  quatx_sub,quaty_sub,quatz_sub,quatw_sub;
    ros::Subscriber linaccelx_sub,linaccely_sub,linaccelz_sub;
    ros::Subscriber rawgyrox_sub,rawgyroy_sub,rawgyroz_sub;
    ros::Subscriber timestamp_sub;
    ros::Subscriber Roll_sub,Pich_sub,Yaw_sub;
    ros::Publisher sensor;
    
    sensor_msgs::Imu imu_data;

 
    msgsConverter(ros::NodeHandle* nh)
    {
        sensor = nh->advertise<sensor_msgs::Imu>("imu_data",1);
        Roll_sub = nh->subscribe("navx/roll",1,RollCallback);
        Pich_sub = nh->subscribe("navx/pitch",1,PitchCallback);
        Yaw_sub = nh->subscribe("navx/yaw",1,YawCallback);
        timestamp_sub = nh->subscribe("navx/last_sensor_timestamp",1,timestampCallback);
        quatx_sub = nh->subscribe("navx/quaternion/x",1,quatxCallback);
        quaty_sub = nh->subscribe("navx/quaternion/y",1,quatyCallback);
        quatz_sub = nh->subscribe("navx/quaternion/z",1,quatzCallback);
        quatw_sub = nh->subscribe("navx/quaternion/w",1,quatwCallback);
        linaccelx_sub = nh-> subscribe("navx/linear_accel/x",1,linaccelxCallback);
        linaccely_sub = nh-> subscribe("navx/linear_accel/y",1,linaccelyCallback);
        linaccelz_sub = nh-> subscribe("navx/linear_accel/z",1,linaccelzCallback);
        rawgyrox_sub = nh->subscribe("navx/raw_gyro/x",1,rawgyroxCallback);
        rawgyroy_sub = nh->subscribe("navx/raw_gyro/y",1,rawgyroyCallback);
        rawgyroz_sub = nh->subscribe("navx/raw_gyro/z",1,rawgyrozCallback);
        // reset_navx_client = nh->serviceClient<std_srvs::Trigger>("reset_navx");

    }

    void msgpub()
    {
        //  std_srvs::Trigger msg1;
        //  reset_navx_client.call(msg1);
        ros::Rate loop_rate(1);
        while(ros::ok())
        {
        imu_data.header.stamp= ros::Time::now();
        imu_data.header.frame_id = "imu_link";
        imu_data.header.seq = seq;
        imu_data.linear_acceleration.x = linaccelx;
        imu_data.linear_acceleration.y = linaccely;
        imu_data.linear_acceleration.z = linaccelz;
        imu_data.angular_velocity.x = rawgyrox;
        imu_data.angular_velocity.y = rawgyroy;
        imu_data.angular_velocity.z = rawgyroz;
        imu_data.orientation.x = quatx*-1;
        imu_data.orientation.y = quaty*-1;
        imu_data.orientation.z = quatz*-1;
        imu_data.orientation.w = quatw*-1;
        sensor.publish(imu_data);
        ros::spinOnce();
        loop_rate.sleep();
        }
    }
    

};

int main(int argc,char **argv)
{
    
    //system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
    ROS_INFO_STREAM("sensor thread: " << syscall(SYS_gettid));
   ros::init(argc, argv, "msgConverter");

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); 
    VMXErrorCode vmxerr;

    ros::AsyncSpinner spinner(6);
    spinner.start();

   navXROSWrapper navx(&nh, &vmx);
   msgsConverter mcf(&nh);
   mcf.msgpub();
   
    ros::waitForShutdown();
    return 0;
}