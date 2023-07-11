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
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static double motoer1encoder,motoer0encoder,motoer2encoder;
static double motor1_speed,motor0_speed,motor2_speed;
static double rightSpeed , leftSpeed , backSpeed;
double teleoperated_x, teleoperated_y,teleoperated_z;
static double angle,angle_t;
static double quatx,quaty,quatz,quatw;
static double linaccelx,linaccely,linaccelz;
static double rawgyrox,rawgyroy,rawgyroz;
static double timestamp;
static double Roll,Pitch,Yaw;
static double seq = 0;
static double wheelRadius = 52;

void motor1_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motor1_speed = msg->data;
}
void motor0_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motor0_speed = msg->data;
}
void motor2_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motor2_speed = msg->data;
}

void  teleoperated_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
   teleoperated_x=msg->linear.x; 
   teleoperated_y=msg->linear.y;
   teleoperated_z=msg->angular.z;
}
void encoder1_dis_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motoer1encoder = msg->data;
}

void encoder0_dis_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motoer0encoder = msg->data;
}
void encoder2_dis_callback(const std_msgs::Float32::ConstPtr& msg)
{
   motoer2encoder = msg->data;
}

void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle = abs(msg->data);
}
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
    rawgyrox = msg->data *(2*M_PI*wheelRadius)/360;

}
void rawgyroyCallback(const std_msgs::Float32::ConstPtr& msg)
{
    rawgyroy = msg->data *(2*M_PI*wheelRadius)/360;

}
void rawgyrozCallback(const std_msgs::Float32::ConstPtr& msg)
{
    rawgyroz = msg->data *(2*M_PI*wheelRadius)/360;

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
class TeleopControl
{
   private:
  
   public:
      double TransformationMatrix[3][3];
      double tempTransformationMatrix[3][3];
      double result[3][3];
      double newx;
      double newy;
      double newz;
      double displacey;
      double displacex;
      ros::ServiceClient set_m_speed,resetAngle,res_encoder_client;
      //motor 1 ::right , moto,r 0 ::left , motor 2 :: back
      ros::Subscriber motor1_speed_sub,motor0_speed_sub,motor2_speed_sub; 
      ros::Subscriber encoder1_dis,encoder0_dis,encoder2_dis;
      ros::Subscriber teleoperated_sub ;   
      vmxpi_ros::MotorSpeed speed_msg;
      geometry_msgs::Twist twist_msg;
      ros::Publisher odom_pub;
      tf::TransformBroadcaster odom_broadcaster;
      ros::Subscriber  angle_sub, yawAngle_sub;
      ros::Time current_time, last_time;
      //imu
   
         ros::Subscriber  quatx_sub,quaty_sub,quatz_sub,quatw_sub;
        ros::Subscriber linaccelx_sub,linaccely_sub,linaccelz_sub;
        ros::Subscriber rawgyrox_sub,rawgyroy_sub,rawgyroz_sub;
        ros::Subscriber timestamp_sub;
        ros::Subscriber Roll_sub,Pich_sub,Yaw_sub;
        ros::Publisher sensor;

        tf2::Quaternion quaternion;
        tf::TransformBroadcaster tf_broadcaster;
        ros::Publisher quaternion_pub; 
      sensor_msgs::Imu imu_data;




   
      TeleopControl(ros::NodeHandle *nh){
         set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");  

         motor1_speed_sub = nh->subscribe("titan/motor1/speed", 1, motor1_speed_callback);
         motor0_speed_sub = nh->subscribe("titan/motor0/speed", 1, motor0_speed_callback);
         motor2_speed_sub = nh->subscribe("titan/motor2/speed", 1, motor2_speed_callback);

         teleoperated_sub = nh->subscribe("cmd_vel",10,teleoperated_callback);

         encoder1_dis = nh->subscribe("titan/encoder1/distance",1,encoder1_dis_callback);
         encoder0_dis = nh->subscribe("titan/encoder0/distance",1,encoder0_dis_callback);
         encoder2_dis = nh->subscribe("titan/encoder2/distance",1,encoder2_dis_callback);
         odom_pub = nh->advertise<nav_msgs::Odometry>("odom_pub",50);
        angle_sub = nh->subscribe("navx/angle", 1, angleCallback);
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
        
          sensor = nh->advertise<sensor_msgs::Imu>("imu_data",30);
         resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
      //   identaty of hte transformation matrix
        for(int i =0; i<3; i++)
        {
         for(int j =0; j<3; j++)
         {
            if (i == j)
               TransformationMatrix[i][j]=1;
         }
        }
   
       }

      double avarage_enc_distance()
      {
         return (((motoer1encoder + motoer0encoder)/2)*1.134);
      }

      
      void holonomicDrive(double x, double y, double z)
    {
      //   rightSpeed = (((x/3)- (y/sqrt(3))+z)*sqrt(3));
      //   leftSpeed = (((x/3)+ (y/sqrt(3))+z)*sqrt(3));
      //   backSpeed = (((-2*x/3)+z)*sqrt(3));
         rightSpeed = (x / 2) + (-(y * (sqrt(3) / 2))) + z;
        leftSpeed = (x / 2) + (y * sqrt(3) / 2) + z;
        backSpeed = -x + z;


        double max = abs(rightSpeed);
        if (abs(leftSpeed) > max)
        {
            max = abs(leftSpeed);
        }
        if (abs(backSpeed) > max)
        {
            max = abs(backSpeed);
        }
        if (max > 1)
        {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }



    }
        void encoder2dist()
    {
        //Displace forward and back
        displacey = ((motoer0encoder * (sqrt(3) / 2)) + ((motoer1encoder * (sqrt(3) / 2)) * -1)) * -0.68;
        //displacey = (left_encoder + right_encoder) / -1.5;
        //displacey = left_encoder * (sqrt(3) / 2);
        displacey /= 1000;

        //Dispace left and right
        //displacex = back_encoder;
        displacex = (motoer2encoder + (motoer0encoder * -0.5) + (motoer1encoder * -0.5)) * 0.70;
        displacex /= 1000;

        
    }

       void odompub()
    {
        current_time = ros::Time::now();
   
        
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        
         odom_trans.transform.translation.x = displacex*-1;
         odom_trans.transform.translation.y = displacey*-1;
         odom_trans.transform.translation.z = angle;
         odom_trans.transform.rotation = odom_quat;
             //send the transform
        odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = displacex*-1;
    odom.pose.pose.position.y = displacey*-1;
    odom.pose.pose.position.z =angle;
    odom.pose.pose.orientation = odom_quat;
    
    odom.twist.twist.linear.x = (2/3)*(motor1_speed+motor0_speed+motor2_speed);
    odom.twist.twist.linear.y = (1/3)*(motor1_speed+motor0_speed+motor2_speed);
    odom.twist.twist.angular.z = 0;

       for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      odom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       odom.pose.covariance[i] += 0.1;
     }
     else {
       odom.pose.covariance[i] = 0;
     }
  }
    odom_pub.publish(odom);

    last_time = current_time;
    }

   void motorcontorl()
   {
      speed_msg.request.speed = rightSpeed ;
      speed_msg.request.motor =1;
      set_m_speed.call(speed_msg);
      ROS_INFO("RIGHT SPEED = %f",rightSpeed);

      speed_msg.request.speed = leftSpeed ;
      speed_msg.request.motor =0;
      set_m_speed.call(speed_msg);
      ROS_INFO("LEFT SPEED = %f",leftSpeed);

      speed_msg.request.speed = backSpeed ;
      speed_msg.request.motor =2;
      set_m_speed.call(speed_msg);
      ROS_INFO("BACK SPEED = %f",backSpeed);
   }

    void reset()
    {
        std_srvs::Trigger msg1; 
        res_encoder_client.call(msg1); // Resets displacement encoders
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
    }
     void imu_datapub()
    {
        current_time = ros::Time::now();
                // Broadcast the transform between the base frame and the IMU frame
        tf::Quaternion imu_orientation;
        // Set appropriate values for imu_orientation

        tf::Transform imu_transform;
        

        imu_transform.setOrigin(tf::Vector3(0, 0, 0));  // No translation, assuming the IMU is at the origin
        imu_orientation.setRPY(Roll,Pitch,Yaw);
        imu_transform.setRotation(imu_orientation);
        

        tf_broadcaster.sendTransform(tf::StampedTransform(imu_transform,current_time, "base_footprint", "Imu_link"));

        imu_data.header.stamp= current_time;
        imu_data.header.frame_id = "Imu_link";
        imu_data.header.seq = seq;

        imu_data.linear_acceleration.x = linaccelx;
        imu_data.linear_acceleration.y = linaccely;
        imu_data.linear_acceleration.z = linaccelz;

        imu_data.linear_acceleration_covariance[0] = 0.1;
        imu_data.linear_acceleration_covariance[4] = 0.1;
        imu_data.linear_acceleration_covariance[8] = 0.1;

        imu_data.angular_velocity.x = rawgyrox;
        imu_data.angular_velocity.y = rawgyroy;
        imu_data.angular_velocity.z = rawgyroz;

        imu_data.angular_velocity_covariance[0] = 0.001*(0.001);
        imu_data.angular_velocity_covariance[4] = 0.001*(0.001);
        imu_data.angular_velocity_covariance[8] = 0.001*(0.001);
  
        imu_data.orientation.x = quatx;
        imu_data.orientation.y = quaty;
        imu_data.orientation.z = quatz;
        imu_data.orientation.w = quatw;

         imu_data.orientation_covariance[0] = (0.01);
         imu_data.orientation_covariance[4] = (0.01);
         imu_data.orientation_covariance[8] = (0.01);

        sensor.publish(imu_data);
        last_time = current_time;
        
    }

   void moving_motor_teleop()
   {
      current_time = ros::Time::now();
      ros::Rate loop_rate(1);
      reset();
      
      while (ros::ok())
      {
         holonomicDrive(teleoperated_x,-teleoperated_y,teleoperated_z);
           //since all odometry is 6DOF we'll need a quaternion created from yaw
           
            motorcontorl(); 
            encoder2dist();
            imu_datapub(); 
            odompub();

    last_time = current_time;
      ros::spinOnce();
      loop_rate.sleep();
         }
   }
   
   
};

// Returns the speed of motor 1


int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "drive_keyboard_teleop_node");

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
   ros::AsyncSpinner spinner(4);
   spinner.start();

   TitanDriverROSWrapper titan(&nh, &vmx);
   
   navXROSWrapper navx(&nh, &vmx);
   TeleopControl cfg(&nh);
   cfg.moving_motor_teleop();
   
   
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();

}