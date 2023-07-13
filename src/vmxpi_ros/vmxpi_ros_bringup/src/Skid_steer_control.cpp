
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

//  M3                 M2

//  M1                M0
static double left_back_enc,right_back_enc,right_front_enc,left_front_enc;
static double left_back_speed,right_back_speed,right_front_speed,left_front_speed;
static double left_back_rpm,right_back_rpm,right_front_rpm,left_front_rpm;
static double left_wheel_speed,right_wheel_speed;
static double distance_left_wheel=0,distance_right_wheel=0;
 bool MagnitudeAtStepoint = false;
static double teleoperated_x, teleoperated_y,teleoperated_z;
// the parameter that used in skid_steerDrive finction 
static double displacey = 0,displacex = 0,theta = 0;
static double prev_right_back_enc = 0.0;
static double prev_right_front_enc = 0.0;
static double prev_left_front_enc = 0.0;
static double prev_left_back_enc = 0.0;
static double  control_output=0;
const int dw = 222; //mm 170
ros::Time previousTime;

static double angle,angle_t;
static double quatx,quaty,quatz,quatw;
static double linaccelx,linaccely,linaccelz;
static double rawgyrox,rawgyroy,rawgyroz;
static double timestamp;
static double Roll,Pitch,Yaw;
static double seq = 0;
static double wheelRadius = 52;
const double track_width = 0.321;  // Physical width of the robot
const double wheelbase = 0.205;  // Physical length of the robot
const double wheel_radius = 0.0626; // Radius of the wheels
const double k_angular = 1.5;
     
void left_back_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   left_back_speed = msg->data;
}
void right_back_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   right_back_speed = msg->data;
}
void right_front_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   right_front_speed = msg->data;
}

void left_front_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
   left_front_speed = msg->data;
}

void left_back_rpm_callback(const std_msgs::Int16::ConstPtr& msg)
{
   left_back_rpm = msg->data*-1;
}
void right_back_rpm_callback(const std_msgs::Int16::ConstPtr& msg)
{
   right_back_rpm = msg->data;
}
void right_front_rpm_callback(const std_msgs::Int16::ConstPtr& msg)
{
   right_front_rpm = msg->data;
}

void left_front_rpm_callback(const std_msgs::Int16::ConstPtr& msg)
{
   left_front_rpm = msg->data*-1;
}

void  teleoperated_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
   teleoperated_x=msg->linear.x; 
   teleoperated_y=msg->linear.y;
   teleoperated_z=msg->angular.z;
}
void left_back_enc_sub_callback(const std_msgs::Float32::ConstPtr& msg)
{
   left_back_enc = msg->data;
}

void right_back_enc_sub_callback(const std_msgs::Float32::ConstPtr& msg)
{
   right_back_enc = msg->data*-1;
}
void right_front_enc_sub_callback(const std_msgs::Float32::ConstPtr& msg)
{
   right_front_enc = msg->data*-1;
}

void left_front_enc_sub_callback(const std_msgs::Float32::ConstPtr& msg)
{
   left_front_enc = msg->data;
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
    Yaw = msg->data*-1;

}

void timestampCallback(const std_msgs::UInt64::ConstPtr& msg)
{
    timestamp = msg->data;

}


class VelocityPlanner {
public:
    VelocityPlanner() {
        
    }

    std::vector<double> generateVelocityProfile(double current_velocity, double target_velocity, double duration) {
        double delta_v = target_velocity - current_velocity;

        // Calculate the required acceleration to reach the target velocity.
        double required_acceleration = delta_v / duration;

        // Apply acceleration and ramping constraints to the required acceleration.
        double acceleration = std::min(std::max(required_acceleration, -a_max), a_max);

        // Calculate the time required to reach the target velocity in the acceleration phase.
        double time_accel = std::abs(delta_v / acceleration);

        // Calculate the time required to reach the target velocity in the deceleration phase.
        double time_decel = std::abs(target_velocity / acceleration);

        // Calculate the time spent at the target velocity (constant velocity phase).
        double time_const_vel = duration - 2 * time_accel;

        // Calculate the time step and the number of steps based on the duration and a small time increment (e.g., 0.1 seconds).
        double dt = 0.1;
        int num_steps = std::ceil(duration / dt);

        std::vector<double> velocity_profile;
        for (int i = 0; i < num_steps; i++) {
            double time = dt * i;
            double velocity;

            if (time < time_accel) {
                // Acceleration phase
                velocity = current_velocity + acceleration * time;
            } else if (time >= time_accel && time < (time_accel + time_const_vel)) {
                // Constant velocity phase
                velocity = target_velocity;
            } else {
                // Deceleration phase
                double deceleration_time = time - time_accel - time_const_vel;
                velocity = target_velocity - acceleration * deceleration_time;
            }

            velocity_profile.push_back(velocity);
        }

        return velocity_profile;
    }

private:
    // Define your parameters here (e.g., v_max, a_max).
    double v_max = 0.5; // Replace with your robot's maximum velocity.
    double a_max = 0.2; // Replace with your robot's maximum acceleration.
};



class PIDController {
public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0), prev_error_(0) {}

    double calculateControlOutput(double error, double dt) {
        // PID control computation.
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        double control_output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // Update the previous error for the next iteration.
        prev_error_ = error;

        return control_output;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double prev_error_;
};

class SkidSterController {
    private:

   bool flag = true;
    double integrator, prevError, differentiator, prevMeasurement, output;
    double prev_angle = 0.0;
    int increment = 1.3;

    public:
    // classes
    VelocityPlanner velocity_planner_;
    PIDController pid_controller_;

          ros::Publisher displacex_pub, displacey_pub, magnitude_pub, lmotor_PID_pub, rmotor_PID_pub, bmotor_PID_pub, error_pub;
      ros::ServiceClient set_m_speed,resetAngle,res_encoder_client,stop_motors_client,enable_client,disable_client;
      //motor 1 ::right , moto,r 0 ::left , motor 2 :: back
      ros::Subscriber left_back_speed_sub,right_back_speed_sub,right_front_speed_sub,left_front_speed_sub; 
      ros::Subscriber left_back_rpm_sub,right_back_rpm_sub,right_front_rpm_sub,left_front_rpm_sub; 
      ros::Subscriber left_back_enc_sub,right_back_enc_sub,right_front_enc_sub,left_front_enc_sub;
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
  
         double tau = 0.02, T = 0.02;
    //double tolerance = 1.0; // Tolerance of Encoder distances(mm) and angle(deg)
         double kP, kI, kD, error;
         double limMin = -0.7, limMax = 0.7, limMinInt = -0.5, limMaxInt = 0.5;
         bool atSetpoint;

    SkidSterController (ros::NodeHandle *nh):velocity_planner_(),pid_controller_(.5, 0.1, 0.01){
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed"); 
        enable_client = nh->serviceClient<std_srvs::Trigger>("titan/enable");
        disable_client = nh->serviceClient<std_srvs::Trigger>("titan/disable");
        stop_motors_client = nh->serviceClient<std_srvs::Trigger>("titan/stop_motors"); 

         left_back_speed_sub = nh->subscribe("titan/motor1/speed", 1, left_back_speed_callback);
         right_back_speed_sub = nh->subscribe("titan/motor0/speed", 1, right_back_speed_callback);
         right_front_speed_sub = nh->subscribe("titan/motor2/speed", 1, right_front_speed_callback);
         left_front_speed_sub = nh->subscribe("titan/motor3/speed", 1, left_front_speed_callback);

         left_back_rpm_sub = nh->subscribe("titan/motor1/rpm", 1, left_back_rpm_callback);
         right_back_rpm_sub = nh->subscribe("titan/motor0/rpm", 1, right_back_rpm_callback);
         right_front_rpm_sub = nh->subscribe("titan/motor2/rpm", 1, right_front_rpm_callback);
         left_front_rpm_sub = nh->subscribe("titan/motor3/rpm", 1, left_front_rpm_callback);

         teleoperated_sub = nh->subscribe("cmd_vel",10,teleoperated_callback);

         left_back_enc_sub = nh->subscribe("titan/encoder1/distance",1,left_back_enc_sub_callback);
         right_back_enc_sub = nh->subscribe("titan/encoder0/distance",1,right_back_enc_sub_callback);
         right_front_enc_sub = nh->subscribe("titan/encoder2/distance",1,right_front_enc_sub_callback);
         left_front_enc_sub = nh->subscribe("titan/encoder3/distance",1,left_front_enc_sub_callback);

        displacex_pub = nh->advertise<std_msgs::Float32>("displace_x", 1);
        displacey_pub = nh->advertise<std_msgs::Float32>("displace_y", 1);
        magnitude_pub = nh->advertise<std_msgs::Float32>("magnitude", 1);

        lmotor_PID_pub = nh->advertise<std_msgs::Float32>("lmotor_PID", 1);
        rmotor_PID_pub = nh->advertise<std_msgs::Float32>("rmotor_PID", 1);
        bmotor_PID_pub = nh->advertise<std_msgs::Float32>("bmotor_PID", 1);
        error_pub = nh->advertise<std_msgs::Float32>("error", 1);


         odom_pub = nh->advertise<nav_msgs::Odometry>("odom",50);
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

       void reset()
    {
        std_srvs::Trigger msg1; 
        res_encoder_client.call(msg1); // Resets displacement encoders
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
    }



  void motorcontorl()
   {
      speed_msg.request.speed = left_wheel_speed;
 
   }



          void skid_steerDrive(double x,double z)
    {

    left_wheel_speed = x -  (z*track_width*k_angular)/2;
    right_wheel_speed = (x + (z*track_width*k_angular)/2) ;


        double max = abs(left_wheel_speed);
        if (abs(right_wheel_speed) > max)
        {
            max = abs(right_wheel_speed);
        }


        if (max > 1)
        {
            right_wheel_speed /= max;
            left_wheel_speed /= max;
        }

         ros::Time currentTime = ros::Time::now();
        ros::Duration deltaTime = currentTime - previousTime;
      double deltaT = deltaTime.toSec();// encoder informaiton into angular velocity 
 

  
        double d_right_back_enc = right_back_enc - prev_right_back_enc;
        double d_right_front_enc = right_front_enc - prev_right_front_enc;
        double d_left_front_enc = left_front_enc - prev_left_front_enc;
        double d_left_back_enc = left_back_enc - prev_left_back_enc;

         double dr = (d_right_back_enc+d_right_front_enc)/(2);
         double dl = (d_left_front_enc+d_left_back_enc)/(2);
         double d = (dr+dl)/2;
         double dtheta = ((dr-dl)/(2*dw))*-1;
        
         displacex += d * cos((Yaw *(M_PI/180)) + (dtheta/2));
         displacey += d * sin((Yaw *(M_PI/180)) + (dtheta/2));

         theta +=dtheta;

         ROS_INFO("displacex = %f //",displacex);
         ROS_INFO("displacey = %f //",displacey);
         ROS_INFO("theta = %f //",theta);
         ROS_INFO("thetaDegree = %f //",theta*(180/M_PI));

     
        prev_right_back_enc = right_back_enc;
        prev_right_front_enc = right_front_enc;
        prev_left_front_enc = left_front_enc;
        prev_left_back_enc = left_back_enc;

      previousTime = currentTime;
      

    }

    double linear_v(double magnitude){
        double target_velocity = 0.5;
        double duration = 10.0;
        double magnitude_ = (sqrt((displacey*displacey)+(displacex*displacex)));
        setPID(0.20, 0.01, 0.00);
        double x_drive = calculate(magnitude,magnitude_, 0.001);
       std::vector<double> velocity_profile = velocity_planner_.generateVelocityProfile(x_drive, target_velocity, duration);
        double magnitude_diff = magnitude-magnitude_;
        double dt = 0.1; 
         for (size_t i = 0; i < velocity_profile.size(); i++){
            double error = velocity_profile[i] - x_drive;
            control_output = (pid_controller_.calculateControlOutput(error, dt))*-1;
            double threshold = 5;

            // if (magnitude_diff<=threshold)
            // {
            //     x_drive=0;
            //     control_output=0;
            //     MagnitudeAtStepoint = true;
            // }   
            // return x_drive;
           }

            ROS_INFO("MagnitudeAtStepoint = %s //",MagnitudeAtStepoint ? "true" : "false");
        
               double threshold = 1;
            if (magnitude_diff<=threshold)
            {
                x_drive=0;
                control_output=0; 
                
     
                MagnitudeAtStepoint = true;
            }   
    
            ROS_INFO("magnitude = %f //",magnitude);
             ROS_INFO("magnitude_ = %f",magnitude_ );     
             if (control_output > 0.5)
                control_output = 0.5;

            return control_output;
   
    }
         double drive_angle(double theta)
{
        static double  diff_angle;
        
            current_time = ros::Time::now();
            setPID(0.7, 0.001, 0.0000);
            double angle_drive = calculate(theta,Yaw, 0.001);

            if (theta>0)
            {
                 diff_angle = theta-Yaw;
                 ROS_INFO("angle = %f //",angle);
             ROS_INFO("diff_angle = %f",diff_angle );     
             double threshold = 0.5;
            if (diff_angle<=threshold)
            {
                 angle_drive=0;
                flag = true;
         
            }  
            }
            else
            {
                 diff_angle = theta-Yaw;
                 ROS_INFO("angle = %f //",angle);
             ROS_INFO("diff_angle = %f",diff_angle );
             double threshold = -0.5;
            if (diff_angle>=threshold)
                angle_drive=0;
                flag = true;
         
            }
             last_time = current_time;
            return angle_drive;
        
        }


void drive_robot(double goalx,double goaly)
{

        reset();
        double magnitude =(sqrt((goalx*goalx)+(goaly*goaly)));
        double direction = atan2(goaly, goalx) * (180/M_PI);
        

        ros::Rate loop_rate(30);
        while (ros::ok())
        {  
            skid_steerDrive(0.0, 0.0); //Check step to zero the motors

          
            skid_steerDrive(linear_v(magnitude), drive_angle(direction));
            motorcontorl();
           
             ROS_INFO("linear_v = %f //",linear_v(magnitude));
            ROS_INFO("drive_angle = %f //",drive_angle(direction));  
           

            if ( MagnitudeAtStepoint == true)
            {
             skid_steerDrive(0, 0);

                PIDReset();
         ros::Duration(1.0).sleep();
                reset();
                break;
            }  

            //  last_time = current_time;
 
        last_time = current_time;
        ros::spinOnce();
        loop_rate.sleep();
        }
       
        }


};

int main(int argc, char **argv)
{
   system("/usr/local/frc/bin/frcKillRobot.sh"); //Terminal call to kill the robot manager used for WPILib before running the executable.
   ros::init(argc, argv, "Skid_steer_controle_node");

   ros::NodeHandle nh; //internal reference to the ROS node that the program will use to interact with the ROS system
   VMXPi vmx(true, (uint8_t)50); //realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
   ros::AsyncSpinner spinner(5);
   spinner.start();
    previousTime = ros::Time::now();
   TitanDriverROSWrapper titan(&nh, &vmx);
   navXROSWrapper navx(&nh, &vmx);

   SkidSterController cfg(&nh);
   cfg.drive_robot(1200,0);
    ros::Duration(1.0).sleep(); // sleep for 1 second
    cfg.drive_robot(1200,500);
   ROS_INFO("ROS SHUTDOWN");
   ros::waitForShutdown();

}