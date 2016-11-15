#include <ros/ros.h>
#include <ros/publisher.h>
#include <rr_platform/speed.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

float speed_control = 0; //command sent to robot by planner
int state_prediction = 0;//unrefined prediction of robot state
int state_covariance;
float accel_x = 0; //acceleration measured by the IMU
float last_accel_x = 0; //the last acceleration measured by the IMU
float measured_vel = 0; //velocity measured by the encoder
float kalman_gain = 0; 
float state; //best estimate of state
float imu_vel = 0; //integrated acceleration from the IMU

void speed_update(const rr_platform::speed::ConstPtr &msg) {
    speed_control = msg->speed; 
}

void data_update(const sensor_msgs::Imu::ConstPtr &msg) {
    accel_x =  (float) msg->linear_acceleration.x;
    //trapezoidal riemann sum to approximate integral of velocity
    imu_vel += (((accel_x + last_accel_x)/2) * 0.2); 
}

void vel_update(const std_msgs::Float64::ConstPtr &msg) {
    measured_vel = (float) msg->data;
}

int computePrediction(int speed) {
    //v = v0 + at
    return speed + (accel_x * 0.02);
}

//combine gaussians representing prediction, the encoder, and the IMU
float fuse_data(float prediction_mean, float imu_mean, float encoder_mean) {
    float fused_data;
    fused_data = prediction_mean + (kalman_gain * (encoder_mean - state_prediction));
    fused_data += kalman_gain * (imu_mean - fused_data);
    return fused_data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalmanfilter");

    ros::NodeHandle nh;
    auto speed_sub = nh.subscribe("/speed",1, speed_update);
    auto imu_sub = nh.subscribe("/imu/data_raw",1,data_update);
    auto enc_sub = nh.subscribe("/encoder_vel",1, vel_update);

    ros::Publisher state_pub = nh.advertise<std_msgs::Float64>("/state", 1);

    ros::Rate rate(20);
    std_msgs::Float64 state_estimate;
    while(ros::ok()) {
        ros::spinOnce();

        state_prediction = computePrediction(speed_control);

        state_estimate.data = fuse_data(state_prediction, imu_vel, measured_vel);
        state_pub.publish(state_estimate);
        last_accel_x = accel_x;

        rate.sleep();
    }
    return 0;    
}
