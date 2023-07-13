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

#include <dynamic_reconfigure/server.h>
#include <vmxpi_ros_bringup/MotorSpeedConfig.h>
#include <std_msgs/Float32.h>
#include <vmxpi_ros/Float.h>
#include <cmath>

#include <time.h>

static double leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed; 
static double leftFrontSpeed_Desired, rightFrontSpeed_Desired, leftBackSpeed_Desired, rightBackSpeed_Desired;
static double leftFront_count, rightFront_count, leftBack_count, rightBack_count;
static double vx_local, vth_local, angle, angle_t;
static double PI = 3.14159265;
static double l = 150;



// Callbacks for Encoder Distance values
void motor0Callback(const std_msgs::Float32::ConstPtr& msg){rightFrontSpeed = msg->data;}
void motor1Callback(const std_msgs::Float32::ConstPtr& msg){rightBackSpeed = msg->data;}
void motor2Callback(const std_msgs::Float32::ConstPtr& msg){leftFrontSpeed = msg->data;}
void motor3Callback(const std_msgs::Float32::ConstPtr& msg){leftBackSpeed = msg->data;}
void angleCallback(const std_msgs::Float32::ConstPtr& msg){angle = abs(msg->data);}
void yawCallback(const std_msgs::Float32::ConstPtr& msg){angle_t = msg->data;}

// Callbacks for Encoder count values
void enc0Callback(const std_msgs::Int32::ConstPtr& msg){rightFront_count = msg->data;}
void enc1Callback(const std_msgs::Int32::ConstPtr& msg){rightBack_count = msg->data;}
void enc2Callback(const std_msgs::Int32::ConstPtr& msg){leftFront_count = msg->data;}
void enc3Callback(const std_msgs::Int32::ConstPtr& msg){leftBack_count = msg->data;}

class DynamicReconfig {
    bool flag = true;
    double integrator, prevError, differentiator, prevMeasurement, output;
    double prev_angle = 0.0;

public:
    ros::ServiceClient set_m_speed, enable_client, disable_client;
    ros::ServiceClient resetAngle, res_encoder_client, stop_motors_client;

    ros::Subscriber lmotor_sub, rmotor_sub, bmotor_sub;
    ros::Subscriber motor0speed_pub, motor1speed_pub, motor2speed_pub, motor3speed_pub, angle_sub, yawAngle_sub;
    ros::Publisher vx_local_pub, vth_local_pub, magnitude_pub, error_pub;

    double tau = 0.02, T = 0.02;
    //double tolerance = 1.0; // Tolerance of Encoder distances(mm) and angle(deg)
    double kP, kI, kD, error;
    double limMin = -1.0, limMax = 1.0, limMinInt = -0.5, limMaxInt = 0.5;
    bool atSetpoint;

    DynamicReconfig(ros::NodeHandle *nh) {
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");

        motor0speed_pub = nh->subscribe("titan/motor0/speed", 1, motor0Callback);
        motor1speed_pub = nh->subscribe("titan/motor1/speed", 1, motor1Callback);
        motor2speed_pub = nh->subscribe("titan/motor2/speed", 1, motor2Callback);
        motor3speed_pub = nh->subscribe("titan/motor3/speed", 1, motor3Callback);
        angle_sub = nh->subscribe("navx_local/angle", 1, angleCallback);
        yawAngle_sub = nh->subscribe("navx_local/yaw", 1, yawCallback);

        vx_local_pub = nh->advertise<std_msgs::Float32>("vx_local", 1);
        vth_local_pub = nh->advertise<std_msgs::Float32>("vth_local", 1);
        magnitude_pub = nh->advertise<std_msgs::Float32>("magnitude", 1);

        error_pub = nh->advertise<std_msgs::Float32>("error", 1);

        enable_client = nh->serviceClient<std_srvs::Trigger>("titan/enable");
        disable_client = nh->serviceClient<std_srvs::Trigger>("titan/disable");

        resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx_local");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");
        stop_motors_client = nh->serviceClient<std_srvs::Trigger>("titan/stop_motors");
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

    void DirectKinematics()
    {
        double Vright = ( rightFrontSpeed + rightBackSpeed ) / 2;
        double Vleft = ( leftFrontSpeed + leftBackSpeed ) / 2;

        vx_local = (Vright + Vleft) / 2;
        vth_local = (Vright - Vleft) / (2 * l);
    }

    void PubVelocity()
    {
        std_msgs::Float32 msg;
        msg.data = 1;
        vx_local_pub.publish(msg);
        msg.data = vth_local;
        vth_local_pub.publish(msg);

        msg.data = error;
        error_pub.publish(msg);
    }

    void InverseKinematis(double v, double w)
    {
        double Vr = ((2 * v) + (w * 2 *l)) / 2;
        double Vl = ((2 * v) - (w * 2 *l)) / 2;

        leftFrontSpeed_Desired = Vl;
        leftBackSpeed_Desired = Vl;
        rightFrontSpeed_Desired = Vr;
        rightBackSpeed_Desired = Vr;
    }

    void reset()
    {
        std_srvs::Trigger msg1;
        stop_motors_client.call(msg1); // Stops motors
        res_encoder_client.call(msg1); // Resets displacement encoders
        std_srvs::Empty msg2;
        resetAngle.call(msg2); // Resets yaw variable
    }

    void setPID(double Kp, double Ki, double Kd)
    {
        kP = Kp;
        kI = Ki;
        kD = Kd;
    }
    
    void stop_motors()
    {
        vmxpi_ros::MotorSpeed msg1;

        msg1.request.speed = 0.0;
        msg1.request.motor = 0;
        set_m_speed.call(msg1);

        msg1.request.speed = 0.0;
        msg1.request.motor = 1;
        set_m_speed.call(msg1);

        msg1.request.speed = 0.0;
        msg1.request.motor = 2;
        set_m_speed.call(msg1);

        msg1.request.speed = 0.0;
        msg1.request.motor = 3;
        set_m_speed.call(msg1);
    }
    
    void setMovements(double magnitude, double Angle)
    {
        if (prev_angle > Angle)
        {
            Angle = Angle - prev_angle + 360.0;
        }
        else
            Angle = Angle - prev_angle;

        ros::Rate loop_rate(50);
        while (ros::ok())
        {
            InverseKinematis(0.0, 0.0); //Check step to zero the motors

            if (flag == true)
            {
                reset();
                angle = 0;
                PIDReset();
                flag = false;
            }
            
            setPID(0.95, 0.0, 0.001);
            double x_drive = calculate(magnitude, vx_local, 0.001);
                        
            setPID(0.050, 0.0, 0.0);
            double angle_drive = calculate(Angle, vth_local, 0.0);

            InverseKinematis(x_drive, angle_drive);
            
            publish_motors();
            DirectKinematics();
            PubVelocity();
            
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void publish_motors()
    {
        vmxpi_ros::MotorSpeed msg1;

        msg1.request.speed = rightFrontSpeed_Desired;
        msg1.request.motor = 0;
        set_m_speed.call(msg1);

        msg1.request.speed = rightBackSpeed_Desired;
        msg1.request.motor = 1;
        set_m_speed.call(msg1);

        msg1.request.speed = leftFrontSpeed_Desired;
        msg1.request.motor = 2;
        set_m_speed.call(msg1);

        msg1.request.speed = leftBackSpeed_Desired;
        msg1.request.motor = 3;
        set_m_speed.call(msg1);
    }


};

int main(int argc, char **argv)
{
    system("/usr/local/frc/bin/frcKillRobot.sh"); // Terminal call to kill the robot manager used for WPILib before running the executable.
    ros::init(argc, argv, "Robot_Interface_node");

    ros::NodeHandle nh;           // internal reference to the ROS node that the program will use to interact with the ROS system
    VMXPi vmx(true, (uint8_t)50); // realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
    ros::AsyncSpinner spinner(4);
    spinner.start();


    TitanDriverROSWrapper titan(&nh, &vmx);
    
    navXROSWrapper navx(&nh, &vmx);
    DynamicReconfig cfg(&nh);

    cfg.setMovements(1.0,0);

    ROS_INFO("ROS SHUTDOWN");
    ros::waitForShutdown();
}