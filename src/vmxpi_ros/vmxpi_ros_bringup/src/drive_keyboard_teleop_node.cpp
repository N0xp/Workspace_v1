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

//  M3                 M2

//  M1                M0
struct MotorCharacteristics
{
    double max_voltage;
    double min_voltage;
};

// Define the characteristics for each motor (update these values based on calibration)
MotorCharacteristics motor0_characteristics = {1, -1};
MotorCharacteristics motor1_characteristics = {1, -1};
MotorCharacteristics motor2_characteristics = {1, -1};
MotorCharacteristics motor3_characteristics = {1, -1};

static double cumulative_error_ = 0.0;
static double previous_error_ = 0.0;
static double voltage_motor0 = 0;
static double voltage_motor1 = 0;
static double voltage_motor2 = 0;
static double voltage_motor3 = 0;
static double desired_speed_mps;
static double control_signal_motor0;
static double control_signal_motor1;
static double control_signal_motor2;
static double control_signal_motor3;
const double max_speed_rpm = 100;
const double Kp = 1.0;  // Proportional gain
const double Ki = 0.1;  // Integral gain
const double Kd = 0.01; // Derivative gain

static double left_back_enc, right_back_enc, right_front_enc, left_front_enc;
static double left_back_speed, right_back_speed, right_front_speed, left_front_speed;
static double left_back_rpm, right_back_rpm, right_front_rpm, left_front_rpm;
static double motor0_count, motor1_count, motor2_count, motor3_count;
static double rwheel_cmd,lwheel_cmd;
static int16_t rwheel_cmd_int16;
static int16_t lwheel_cmd_int16;

static double left_wheel_speed, right_wheel_speed;
static double distance_left_wheel = 0, distance_right_wheel = 0;
static double teleoperated_x, teleoperated_y, teleoperated_z;
static double displacey = 0, displacex = 0, theta = 0;
static double prev_right_back_enc = 0.0;
static double prev_right_front_enc = 0.0;
static double prev_left_front_enc = 0.0;
static double prev_left_back_enc = 0.0;
ros::Time previousTime;
const double wheelRadius = 63;
static double angle, angle_t;
static double quatx, quaty, quatz, quatw;
static double linaccelx, linaccely, linaccelz;
static double rawgyrox, rawgyroy, rawgyroz;
static double timestamp;
static double Roll, Pitch, Yaw;
static double seq = 0;
static double yaw_deg = 0;

const int dw = 210;                // mm 170
const double track_width = 0.321;  // Physical width of the robot
const double wheelbase = 0.205;    // Physical length of the robot
const double wheel_diameter = 410;
const double wheel_radius = 0.063; // Radius of the wheels mm 
const double k_angular = 2.5;
const double CountperMeter = 3698.45;

const int encoder_resolution = 12; // ppr
void left_back_speed_callback(const std_msgs::Float32::ConstPtr &msg)
{
    left_back_speed = msg->data;
}
void right_back_speed_callback(const std_msgs::Float32::ConstPtr &msg)
{
    right_back_speed = msg->data;
}
void right_front_speed_callback(const std_msgs::Float32::ConstPtr &msg)
{
    right_front_speed = msg->data;
}

void left_front_speed_callback(const std_msgs::Float32::ConstPtr &msg)
{
    left_front_speed = msg->data;
}

void left_back_rpm_callback(const std_msgs::Int16::ConstPtr &msg)
{
    left_back_rpm = msg->data * -1;
}
void right_back_rpm_callback(const std_msgs::Int16::ConstPtr &msg)
{
    right_back_rpm = msg->data;
}
void right_front_rpm_callback(const std_msgs::Int16::ConstPtr &msg)
{
    right_front_rpm = msg->data;
}

void left_front_rpm_callback(const std_msgs::Int16::ConstPtr &msg)
{
    left_front_rpm = msg->data * -1;
}

void teleoperated_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    teleoperated_x = msg->linear.x;
 
    teleoperated_z = msg->angular.z;
}
void left_back_enc_sub_callback(const std_msgs::Float32::ConstPtr &msg)
{
    left_back_enc = msg->data;
}

void right_back_enc_sub_callback(const std_msgs::Float32::ConstPtr &msg)
{
    right_back_enc = msg->data * -1;
}
void right_front_enc_sub_callback(const std_msgs::Float32::ConstPtr &msg)
{
    right_front_enc = msg->data * -1;
}

void left_front_enc_sub_callback(const std_msgs::Float32::ConstPtr &msg)
{
    left_front_enc = msg->data;
}

// Callbacks for Encoder count values
void enc0_count_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    motor0_count = msg->data;
}
void enc1_count_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    motor1_count = msg->data;
}
void enc2_count_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    motor2_count = msg->data;
}
void enc3_count_Callback(const std_msgs::Int32::ConstPtr &msg)
{
    motor3_count = msg->data;
}

void rwheel_cmd_Callback(const std_msgs::Float32::ConstPtr &msg)
{
     rwheel_cmd = msg->data;
      rwheel_cmd_int16 = static_cast<int16_t>(rwheel_cmd);
}
void lwheel_cmd_Callback(const std_msgs::Float32::ConstPtr &msg)
{
     lwheel_cmd = msg->data;
     lwheel_cmd_int16 = static_cast<int16_t>(lwheel_cmd);
}

void angleCallback(const std_msgs::Float32::ConstPtr &msg)
{
    angle = abs(msg->data);
}
void quatxCallback(const std_msgs::Float64::ConstPtr &msg)
{
    quatx = msg->data;
}

void quatyCallback(const std_msgs::Float64::ConstPtr &msg)
{
    quaty = msg->data;
}

void quatzCallback(const std_msgs::Float64::ConstPtr &msg)
{
    quatz = msg->data;
}

void quatwCallback(const std_msgs::Float64::ConstPtr &msg)
{
    quatw = msg->data;
}

void linaccelxCallback(const std_msgs::Float32::ConstPtr &msg)
{
    linaccelx = msg->data * 9.80;
}

void linaccelyCallback(const std_msgs::Float32::ConstPtr &msg)
{
    linaccely = msg->data * 9.80;
}

void linaccelzCallback(const std_msgs::Float32::ConstPtr &msg)
{
    linaccelz = msg->data * 9.80;
}

void rawgyroxCallback(const std_msgs::Float32::ConstPtr &msg)
{
    rawgyrox = msg->data * (2 * M_PI * wheelRadius) / 360;
}
void rawgyroyCallback(const std_msgs::Float32::ConstPtr &msg)
{
    rawgyroy = msg->data * (2 * M_PI * wheelRadius) / 360;
}
void rawgyrozCallback(const std_msgs::Float32::ConstPtr &msg)
{
    rawgyroz = msg->data;
}

void RollCallback(const std_msgs::Float32::ConstPtr &msg)
{
    Roll = msg->data;
}

void PitchCallback(const std_msgs::Float32::ConstPtr &msg)
{
    Pitch = msg->data;
}

void YawCallback(const std_msgs::Float32::ConstPtr &msg)
{
    Yaw = msg->data*-1;
}

void timestampCallback(const std_msgs::UInt64::ConstPtr &msg)
{
    timestamp = msg->data;
}

class TeleopControl
{
private:
    bool flag = true;
    double integrator, prevError, differentiator, prevMeasurement, output;
    double prev_angle = 0.0;
    int increment = 1.3;

public:
    double TransformationMatrix[3][3];
    double tempTransformationMatrix[3][3];
    double result[3][3];
    double newx;
    double newy;
    double newz;
    ros::ServiceClient set_m_speed, resetAngle, res_encoder_client;
    // motor 1 ::right , moto,r 0 ::left , motor 2 :: back
    ros::Subscriber left_back_speed_sub, right_back_speed_sub, right_front_speed_sub, left_front_speed_sub;
    ros::Subscriber left_back_rpm_sub, right_back_rpm_sub, right_front_rpm_sub, left_front_rpm_sub;
    ros::Subscriber left_back_enc_sub, right_back_enc_sub, right_front_enc_sub, left_front_enc_sub;

    ros::Subscriber enc0_count_sub, enc3_count_sub, enc1_count_sub, enc2_count_sub;
    ros::Publisher lwheel_count_pub, rwheel_count_pub;
    ros::Subscriber rwheel_cmd_sub,lwheel_cmd_sub;

    ros::Subscriber teleoperated_sub;
    vmxpi_ros::MotorSpeed speed_msg;
    geometry_msgs::Twist twist_msg;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Subscriber angle_sub, yawAngle_sub;
    ros::Time current_time, last_time;
    // imu

    ros::Subscriber quatx_sub, quaty_sub, quatz_sub, quatw_sub;
    ros::Subscriber linaccelx_sub, linaccely_sub, linaccelz_sub;
    ros::Subscriber rawgyrox_sub, rawgyroy_sub, rawgyroz_sub;
    ros::Subscriber timestamp_sub;
    ros::Subscriber Roll_sub, Pich_sub, Yaw_sub;
    ros::Publisher sensor;

    tf2::Quaternion quaternion;
    tf::TransformBroadcaster tf_broadcaster;
    ros::Publisher quaternion_pub;
    sensor_msgs::Imu imu_data;
    ros::Timer control_timer_;

    double tau = 0.02, T = 0.02;
    // double tolerance = 1.0; // Tolerance of Encoder distances(mm) and angle(deg)
    double kP, kI, kD, error;
    double limMin = -0.7, limMax = 0.7, limMinInt = -0.5, limMaxInt = 0.5;
    bool atSetpoint;

    TeleopControl(ros::NodeHandle *nh)
    {
        set_m_speed = nh->serviceClient<vmxpi_ros::MotorSpeed>("titan/set_motor_speed");

        left_back_speed_sub = nh->subscribe("titan/motor1/speed", 1, left_back_speed_callback);
        right_back_speed_sub = nh->subscribe("titan/motor0/speed", 1, right_back_speed_callback);
        right_front_speed_sub = nh->subscribe("titan/motor2/speed", 1, right_front_speed_callback);
        left_front_speed_sub = nh->subscribe("titan/motor3/speed", 1, left_front_speed_callback);

        left_back_rpm_sub = nh->subscribe("titan/motor1/rpm", 1, left_back_rpm_callback);
        right_back_rpm_sub = nh->subscribe("titan/motor0/rpm", 1, right_back_rpm_callback);
        right_front_rpm_sub = nh->subscribe("titan/motor2/rpm", 1, right_front_rpm_callback);
        left_front_rpm_sub = nh->subscribe("titan/motor3/rpm", 1, left_front_rpm_callback);

        teleoperated_sub = nh->subscribe("cmd_vel", 10, teleoperated_callback);

        left_back_enc_sub = nh->subscribe("titan/encoder1/distance", 1, left_back_enc_sub_callback);
        right_back_enc_sub = nh->subscribe("titan/encoder0/distance", 1, right_back_enc_sub_callback);
        right_front_enc_sub = nh->subscribe("titan/encoder2/distance", 1, right_front_enc_sub_callback);
        left_front_enc_sub = nh->subscribe("titan/encoder3/distance", 1, left_front_enc_sub_callback);


        odom_pub = nh->advertise<nav_msgs::Odometry>("odom", 50);
        angle_sub = nh->subscribe("navx/angle", 1, angleCallback);
        Roll_sub = nh->subscribe("navx/roll", 1, RollCallback);
        Pich_sub = nh->subscribe("navx/pitch", 1, PitchCallback);
        Yaw_sub = nh->subscribe("navx/yaw", 1, YawCallback);
        timestamp_sub = nh->subscribe("navx/last_sensor_timestamp", 1, timestampCallback);
        quatx_sub = nh->subscribe("navx/quaternion/x", 1, quatxCallback);
        quaty_sub = nh->subscribe("navx/quaternion/y", 1, quatyCallback);
        quatz_sub = nh->subscribe("navx/quaternion/z", 1, quatzCallback);
        quatw_sub = nh->subscribe("navx/quaternion/w", 1, quatwCallback);
        linaccelx_sub = nh->subscribe("navx/linear_accel/x", 1, linaccelxCallback);
        linaccely_sub = nh->subscribe("navx/linear_accel/y", 1, linaccelyCallback);
        linaccelz_sub = nh->subscribe("navx/linear_accel/z", 1, linaccelzCallback);
        rawgyrox_sub = nh->subscribe("navx/raw_gyro/x", 1, rawgyroxCallback);
        rawgyroy_sub = nh->subscribe("navx/raw_gyro/y", 1, rawgyroyCallback);
        rawgyroz_sub = nh->subscribe("navx/raw_gyro/z", 1, rawgyrozCallback);

        sensor = nh->advertise<sensor_msgs::Imu>("imu_data", 30);
        resetAngle = nh->serviceClient<std_srvs::Empty>("reset_navx");
        res_encoder_client = nh->serviceClient<std_srvs::Trigger>("titan/reset_encoder");

     
        //   identaty of hte transformation matrix
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (i == j)
                    TransformationMatrix[i][j] = 1;
            }
        }
    }

    double calculatePIDControlSignal(const MotorCharacteristics &motor_characteristics, double setpoint_rpm, double actual_speed_rpm)
    {
        // Calculate the error (difference between setpoint and actual speed)
        double error = setpoint_rpm - actual_speed_rpm;

        // Compute the proportional term of the PID controller
        double proportional_term = Kp * error;

        // Compute the integral term of the PID controller
        // You need to track the cumulative error over time
        // and apply the integral gain to reduce steady-state error
        double integral_term = Ki * cumulative_error_;

        // Compute the derivative term of the PID controller
        // This term accounts for the rate of change of the error
        double derivative_term = Kd * (error - previous_error_);

        // Calculate the control signal (voltage) based on the PID terms
        double control_signal = proportional_term + integral_term + derivative_term;

        // Apply saturation limits to the control signal to avoid excessive voltage
        control_signal = std::max(motor_characteristics.min_voltage, std::min(motor_characteristics.max_voltage, control_signal));

        // Update the cumulative error and previous error for the next iteration
        cumulative_error_ += error;
        previous_error_ = error;

        return control_signal;
    }
    void skid_steerDrive(double x, double z)
    {

        left_wheel_speed = x - (z * track_width * k_angular) / 2;
        right_wheel_speed = (x + (z * track_width * k_angular) / 2);

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
        double deltaT = deltaTime.toSec(); // encoder informaiton into angular velocity

        double d_right_back_enc = right_back_enc - prev_right_back_enc;
        double d_right_front_enc = right_front_enc - prev_right_front_enc;
        double d_left_front_enc = left_front_enc - prev_left_front_enc;
        double d_left_back_enc = left_back_enc - prev_left_back_enc;

        double dr = (d_right_back_enc + d_right_front_enc) / (2);
        double dl = (d_left_front_enc + d_left_back_enc) / (2);
        double d = (dr + dl) / 2;
        double dtheta = ((dr - dl) / (wheel_diameter));

        displacex += d * cos((Yaw * (M_PI / 180))) ;
        displacey += d * sin((Yaw * (M_PI / 180))) ;

        theta += dtheta;

        

        prev_right_back_enc = right_back_enc;
        prev_right_front_enc = right_front_enc;
        prev_left_front_enc = left_front_enc;
        prev_left_back_enc = left_back_enc;

        previousTime = currentTime;
    }



    void odompub()
    {

        current_time = ros::Time::now();
        tf2::Quaternion q;
        q.setRPY(Roll,Pitch,Yaw);
        
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Yaw);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        
        odom_quat.x=quatx;
        odom_quat.y=quaty;
        odom_quat.z=quatz;
        odom_quat.w=quatw;
         odom_trans.transform.translation.x = displacex/1000;
         odom_trans.transform.translation.y = displacey/1000;
         odom_trans.transform.translation.z = 0.0;
         odom_trans.transform.rotation = odom_quat;
             //send the transform
        odom_broadcaster.sendTransform(odom_trans);
        // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Yaw);
        // geometry_msgs::TransformStamped odom_trans;
        // odom_trans.header.stamp = current_time;
        // odom_trans.header.frame_id = "odom";
        // odom_trans.child_frame_id = "base_footprint";

        // odom_trans.transform.translation.x = displacex;
        // odom_trans.transform.translation.y = displacey;
        // odom_trans.transform.translation.z = 0;
        // odom_trans.transform.rotation = odom_quat;
        // // send the transform
        // odom_broadcaster.sendTransform(odom_trans);

        // next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        // set the position
        odom.pose.pose.position.x = displacex;
        odom.pose.pose.position.y = displacey;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = ((left_back_speed + right_back_speed + right_front_speed+left_front_speed)/2)*cos((Yaw * (M_PI / 180)));
        odom.twist.twist.linear.y = ((left_back_speed + right_back_speed + right_front_speed+left_front_speed)/2)*sin((Yaw * (M_PI / 180)));
        odom.twist.twist.angular.z = (((right_back_speed+right_front_speed) - (left_back_speed+left_front_speed)) / (wheel_diameter));
        std::vector<double> pose_cov_list = {0.1, 0.1, 0.1, 0.05, 0.05, 0.05};
        odom.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
        
        odom_pub.publish(odom);

        last_time = current_time;
    }

       void motorcontorl()
       {

          speed_msg.request.speed = left_wheel_speed;
          speed_msg.request.motor =1;
          set_m_speed.call(speed_msg);

          speed_msg.request.speed = -right_wheel_speed;
          speed_msg.request.motor =0;
          set_m_speed.call(speed_msg);

          speed_msg.request.speed = -right_wheel_speed;
          speed_msg.request.motor =2;
          set_m_speed.call(speed_msg);

            speed_msg.request.speed = left_wheel_speed;
          speed_msg.request.motor =3;
          set_m_speed.call(speed_msg);

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

        imu_transform.setOrigin(tf::Vector3(0, 0, 0)); // No translation, assuming the IMU is at the origin
        imu_orientation.setRPY(Roll, Pitch, Yaw);
        imu_transform.setRotation(imu_orientation);

        tf_broadcaster.sendTransform(tf::StampedTransform(imu_transform, current_time, "base_footprint", "Imu_link"));

        imu_data.header.stamp = current_time;
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

        imu_data.angular_velocity_covariance[0] = 0.001 * (0.001);
        imu_data.angular_velocity_covariance[4] = 0.001 * (0.001);
        imu_data.angular_velocity_covariance[8] = 0.001 * (0.001);

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
        ros::Rate loop_rate(50);
        reset();

        while (ros::ok())
        {
            skid_steerDrive(teleoperated_x, teleoperated_z);

             motorcontorl();        
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
    system("/usr/local/frc/bin/frcKillRobot.sh"); // Terminal call to kill the robot manager used for WPILib before running the executable.
    ros::init(argc, argv, "drive_keyboard_teleop_node");

    ros::NodeHandle nh;           // internal reference to the ROS node that the program will use to interact with the ROS system
    VMXPi vmx(true, (uint8_t)50); // realtime bool and the update rate to use for the VMXPi AHRS/IMU interface, default is 50hz within a valid range of 4-200Hz
    ros::AsyncSpinner spinner(4);
    spinner.start();

    previousTime = ros::Time::now();
    TitanDriverROSWrapper titan(&nh, &vmx);
    displacey = 0;
    displacex = 0;
    theta = 0;
    navXROSWrapper navx(&nh, &vmx);
    TeleopControl cfg(&nh);

    cfg.moving_motor_teleop();

    ROS_INFO("ROS SHUTDOWN");
    ros::waitForShutdown();
}