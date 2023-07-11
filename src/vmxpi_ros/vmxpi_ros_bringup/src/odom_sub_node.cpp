
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
#include "TitanDriver_ros_wrapper.h"
#include <dynamic_reconfigure/server.h>
#include <vmxpi_ros_bringup/MotorSpeedConfig.h>
#include <vmxpi_ros/Float.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

      //motor 1 ::right , moto,r 0 ::left , motor 2 :: back
static double motor1_speed,motor0_speed,motor2_speed;
static double rightSpeed , leftSpeed , backSpeed;
double teleoperated_forward, teleoperated_side,teleoperated_rotat;
static double motoer1encoder,motoer0encoder,motoer2encoder;
static double enc1,enc2,enc0;
static double odom_x=0.01,odom_y=1,odom_z=0;
static double left_count, right_count, back_count;
static double displacey,displacex,magnitude_t;
static double PI = 3.14159265;
static double diff_x,diff_y,threshold = 0.1;
// navx variabls 
static double quatx,quaty,quatz,quatw;
static double linaccelx,linaccely,linaccelz;
static double rawgyrox,rawgyroy,rawgyroz;
static double timestamp;
static double Roll,Pitch,Yaw,angle;
static double seq = 0;

void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    angle = abs(msg->data);
}
// void yawCallback(const std_msgs::Float32::ConstPtr& msg)
// {
//     angle_t = msg->data;
// }
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
// imu callbacks
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




class odom_sub_node
{
   private:
   bool flag = true;
    double integrator, prevError, differentiator, prevMeasurement, output;
    double prev_angle = 0.0;

   public:
        // client object 
      ros::ServiceClient set_m_speed;
      ros::ServiceClient resetAngle, res_encoder_client, stop_motors_client, enable_client,disable_client;

        //motor object 
      ros::Subscriber motor1_speed_sub,motor0_speed_sub,motor2_speed_sub; 
      ros::Subscriber encoder1_dis,encoder0_dis,encoder2_dis;      
      ros::Subscriber enc1_sub,enc0_sub,enc2_sub;
    ros::Publisher displacex_pub, displacey_pub, magnitude_pub, lmotor_PID_pub, rmotor_PID_pub, bmotor_PID_pub, error_pub;

       //gemotry_msgs/odom object
        ros::Publisher odom_pub;
        tf::TransformBroadcaster odom_broadcaster;
        // navx object 
        ros::Subscriber angle_sub;
         ros::Subscriber  quatx_sub,quaty_sub,quatz_sub,quatw_sub;
        ros::Subscriber linaccelx_sub,linaccely_sub,linaccelz_sub;
        ros::Subscriber rawgyrox_sub,rawgyroy_sub,rawgyroz_sub;
        ros::Subscriber timestamp_sub;
        ros::Subscriber Roll_sub,Pich_sub,Yaw_sub;
        ros::Publisher sensor;

        tf2::Quaternion quaternion;
        tf::TransformBroadcaster tf_broadcaster;
        ros::Publisher quaternion_pub; 

 
    
    
    // msgs object 
      sensor_msgs::Imu imu_data;
      vmxpi_ros::MotorSpeed speed_msg;
      ros::Time current_time, last_time;
         double tau = 0.02, T = 0.02;
    //double tolerance = 1.0; // Tolerance of Encoder distances(mm) and angle(deg)
         double kP, kI, kD, error;
         double limMin = -1.0, limMax = 1.0, limMinInt = -0.5, limMaxInt = 0.5;
         bool atSetpoint;

      odom_sub_node(ros::NodeHandle *nh){
        current_time = ros::Time::now();
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");  
        resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
        stop_motors_client = nh->serviceClient<std_srvs::Trigger>("titan/stop_motors");
        enable_client = nh->serviceClient<std_srvs::Trigger>("titan/enable");
        disable_client = nh->serviceClient<std_srvs::Trigger>("titan/disable");
       
        // publishing the data to imu_data topic 
        sensor = nh->advertise<sensor_msgs::Imu>("imu_data",30);
      
      //
         motor1_speed_sub = nh->subscribe("titan/motor1/speed", 1, motor1_speed_callback);
         motor0_speed_sub = nh->subscribe("titan/motor0/speed", 1, motor0_speed_callback);
         motor2_speed_sub = nh->subscribe("titan/motor2/speed", 1, motor2_speed_callback);

         encoder1_dis = nh->subscribe("titan/encoder1/distance",1,encoder1_dis_callback);
         encoder0_dis = nh->subscribe("titan/encoder0/distance",1,encoder0_dis_callback);
         encoder2_dis = nh->subscribe("titan/encoder2/distance",1,encoder2_dis_callback);
         odom_pub = nh->advertise<nav_msgs::Odometry>("odom_pub",30);

        
        displacex_pub = nh->advertise<std_msgs::Float32>("displace_x", 1);
        displacey_pub = nh->advertise<std_msgs::Float32>("displace_y", 1);
        magnitude_pub = nh->advertise<std_msgs::Float32>("magnitude", 1);

        lmotor_PID_pub = nh->advertise<std_msgs::Float32>("lmotor_PID", 1);
        rmotor_PID_pub = nh->advertise<std_msgs::Float32>("rmotor_PID", 1);
        bmotor_PID_pub = nh->advertise<std_msgs::Float32>("bmotor_PID", 1);
        error_pub = nh->advertise<std_msgs::Float32>("error", 1);


        // navx subscriber 
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
         angle_sub = nh->subscribe("navx/angle", 1, angleCallback);


      }
      void setPID(double Kp, double Ki, double Kd)
    {
        kP = Kp;
        kI = Ki;
        kD = Kd;
    }

    double calculate(double setPoint, double measurement, double tolerance)
    {
        /*
         * Error
         */
        error = setPoint - measurement;

        /*
         * Setpoint check
         */
        if (abs(error) <= tolerance)
        {
            atSetpoint = true;
            return 0.0;
        }
        else
        {
            atSetpoint = false;
        }

        /*
         * Proportional
         */
        double proportional = kP * error;

        /*
         * Integral
         */
        integrator = integrator + 0.5 * kI * T * (error + prevError);

        /*
         * Anti Wind up
         */
        if (integrator > limMaxInt)
        {
            integrator = limMaxInt;
        }
        else if (integrator < limMinInt)
        {
            integrator = limMinInt;
        }

        /*
         * Band limit derivative
         */
        differentiator = -(2.0 * kD * (measurement - prevMeasurement) + (2.0 * tau - T) * differentiator) / (2.0 * tau + T);

        /*
         * Compute
         */
        output = proportional + integrator + differentiator;

        /*
         * Clamp
         */
        if (output > limMax)
        {
            output = limMax;
        }
        else if (output < limMin)
        {
            output = limMin;
        }

        /*
         * Store variables
         */
        prevError = error;
        prevMeasurement = measurement;

        /*
         * Return final value
         */
        return output;
    }
    void PIDReset()
    {
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
        error = 0.0;
        integrator = 0.0;
        prevError = 0.0;
        differentiator = 0.0;
        prevMeasurement = 0.0;
        atSetpoint = false;
        output = 0.0;
    }


     void holonomicDrive(double x, double y, double z)
    {
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

   void motorcontorl()
   {
      speed_msg.request.speed = rightSpeed ;
      speed_msg.request.motor =1;
      set_m_speed.call(speed_msg);
  

      speed_msg.request.speed = leftSpeed ;
      speed_msg.request.motor =0;
      set_m_speed.call(speed_msg);
 

      speed_msg.request.speed = backSpeed ;
      speed_msg.request.motor =2;
      set_m_speed.call(speed_msg);
    
   }

    void reset()
    {
        std_srvs::Trigger msg1;
        stop_motors_client.call(msg1); // Stops motors
        res_encoder_client.call(msg1); // Resets displacement encoders
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
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

        magnitude_t = sqrt(pow(abs(displacex), 2.0) + pow(abs(displacey), 2.0));
    }
    void PubDisplacements()
    {
        std_msgs::Float32 msg;
        msg.data = displacex;
        displacex_pub.publish(msg);
        msg.data = displacey;
        displacey_pub.publish(msg);
        msg.data = magnitude_t;
        magnitude_pub.publish(msg);

        msg.data = leftSpeed;
        lmotor_PID_pub.publish(msg);

        msg.data = rightSpeed;
        rmotor_PID_pub.publish(msg);

        msg.data = backSpeed;
        bmotor_PID_pub.publish(msg);

        msg.data = error;
        error_pub.publish(msg);
    }

  

    void odompub()
    {
        current_time = ros::Time::now();
        tf2::Quaternion q;
        q.setRPY(Roll,Pitch,Yaw);
        
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        
        odom_quat.x=quatx;
        odom_quat.y=quaty;
        odom_quat.z=quatz;
        odom_quat.w=quatw;
         odom_trans.transform.translation.x = displacex*-1;
         odom_trans.transform.translation.y = displacey*-1;
         odom_trans.transform.translation.z = 0.0;
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
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
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
        // double norm = std::sqrt(imu_data.orientation.x * imu_data.orientation.x +
        //                         imu_data.orientation.y * imu_data.orientation.y +
        //                         imu_data.orientation.z * imu_data.orientation.z +
        //                         imu_data.orientation.w * imu_data.orientation.w);
        imu_data.linear_acceleration.x = linaccelx;
        imu_data.linear_acceleration.y = linaccely;
        imu_data.linear_acceleration.z = linaccelz;

        imu_data.linear_acceleration_covariance[0] = 0.1;
        imu_data.linear_acceleration_covariance[4] = 0.1;
        imu_data.linear_acceleration_covariance[8] = 0.1;

        imu_data.angular_velocity.x = rawgyrox;
        imu_data.angular_velocity.y = rawgyroy;
        imu_data.angular_velocity.z = rawgyroz;

        imu_data.angular_velocity_covariance[0] = 0.001;
        imu_data.angular_velocity_covariance[4] = 0.001;
        imu_data.angular_velocity_covariance[8] = 0.001;
  
        imu_data.orientation.x = quatx;
        imu_data.orientation.y = quaty;
        imu_data.orientation.z = quatz;
        imu_data.orientation.w = quatw;

         imu_data.orientation_covariance[0] = 0.01;
         imu_data.orientation_covariance[4] = 0.01;
         imu_data.orientation_covariance[8] = 0.01;
        sensor.publish(imu_data);
        
    }

    void setMovements(double magnitude, double Angle)
    {
        if (prev_angle > Angle)
        {
            Angle = Angle - prev_angle + 360.0;
        }
        else
            Angle = Angle - prev_angle;

        double target_x_displace = magnitude * cos(abs(Angle) * PI / 180.0);
        double target_y_displace = magnitude * sin(abs(Angle) * PI / 180.0);

        ros::Rate loop_rate(50);
        while (ros::ok())
        {

            holonomicDrive(0.0, 0.0, 0.0); //Check step to zero the motors

            if (flag == true)
            {
                reset();
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                flag = false;
            }
            
            setPID(0.95, 0.0, 0.001);
            double x_drive = calculate(target_x_displace, displacex, 0.001);
            
            //setPID(0.9, 0.0, 0.005);
            double y_drive = calculate(target_y_displace, displacey, 0.001);
            
            setPID(0.050, 0.0, 0.0);
            double angle_drive = 0.02;

            holonomicDrive(x_drive, y_drive, angle_drive);
            
            motorcontorl();
           PubDisplacements();
           odompub();

            if (magnitude_t >= abs(magnitude))
            {
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                reset();
                flag = true;
                break;
            }


            ros::spinOnce();
            loop_rate.sleep();
        }
    }
void drive_odom_msg()
{

        // double magnitude = sqrt(odom_x*odom_x + odom_y*odom_y);
        ros::Rate loop_rate(30);
        while (ros::ok())
        {
            current_time = ros::Time::now();
            holonomicDrive(0.0, 0.0, 0.0); //Check step to zero the motors

            if (flag == true)
            {
                reset();
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                flag = false;
            }
            
            setPID(0.95, 0.0, 0.001);
            double x_drive = calculate(odom_x, displacex, 0.001);
            
            // setPID(0.9, 0.0, 0.005);
            double y_drive = calculate(odom_y, displacey, 0.001);
            
            setPID(0.050, 0.0, 0.0);
            double angle_drive = odom_z;

            holonomicDrive(x_drive, y_drive, angle_drive);
            
            motorcontorl();
            encoder2dist();
            PubDisplacements();
            imu_datapub();
            odompub();

            diff_x = odom_x + displacex;
            diff_y = odom_y + displacey;
            ROS_INFO("diff_x = %f // diff_y = %f",diff_x,diff_y);
            ROS_INFO("odom_x = %f // odom_y = %f",odom_x,odom_y);
            if (abs(diff_x)<=threshold && abs(diff_y)<=threshold)
            {
                displacex = 0;
                displacey = 0;
                angle = 0;
                PIDReset();
                reset();
                flag = true;
                PIDReset();
                reset();
                break;
            }  

            last_time = current_time;
 
              ros::spinOnce();
            loop_rate.sleep();
        }
       
        }
    



void callback(vmxpi_ros_bringup::MotorSpeedConfig &config, uint32_t level) {
        std_srvs::Trigger msg;
        if (config.enabled)
            enable_client.call(msg);
        else
            disable_client.call(msg);
        for (int i = 0; i < 1; i++)
        {
             reset();
             drive_odom_msg();
             ros::Duration(1.5).sleep();
            //  setMovements(1.0, 0.0); // (magnitude (m), angle (deg))
            //  ros::Duration(1.5).sleep(); // sleep for 1 second
             
            //  setMovements(1.0, 90.0);
            //  ros::Duration(1.5).sleep(); // sleep for 1 second
             
            //  setMovements(sqrt(2.0), 270.0);

            
             
        }
    }
};

// Returns the speed of motor 1


int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ROS_INFO_STREAM("odom thread: " << syscall(SYS_gettid));
   ros::init(argc, argv, "drive_enc_node");

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   
  
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
  VMXErrorCode vmxerr;
   ros::AsyncSpinner spinner(6);
   spinner.start();

   TitanDriverROSWrapper titan(&nh, &vmx);
   navXROSWrapper navx(&nh, &vmx);
   odom_sub_node cfg(&nh);
    dynamic_reconfigure::Server<vmxpi_ros_bringup::MotorSpeedConfig> server;
    dynamic_reconfigure::Server<vmxpi_ros_bringup::MotorSpeedConfig>::CallbackType f;
    f = boost::bind(&odom_sub_node::callback, &cfg, _1, _2);
    server.setCallback(f);
    // cfg.drive_odom_msg();
   
   
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();

}
