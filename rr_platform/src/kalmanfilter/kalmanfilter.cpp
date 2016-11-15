#include <ros/ros.h>
#include <ros/publisher.h>
#include <rr_platform/speed.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

float speed_control = 0;
int state_prediction = 0;
int state_covariance;
float accel_x = 0; 
float last_accel_x = 0;
float measured_vel = 0;
float kalman_gain = 0;

void speed_update(const rr_platform::speed::ConstPtr &msg) {
    speed_control = msg->speed; 
}

void data_update(const sensor_msgs::Imu::ConstPtr &msg) {
    accel_x =  (float) msg->linear_acceleration.x;
}

void vel_update(const std_msgs::Float64::ConstPtr &msg) {
    measured_vel = (float) msg->data;
}

int computePrediction(int speed) {
    return speed + (accel_x * 0.02);
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
        state_estimate.data = state_prediction + (kalman_gain * (measured_vel - state_prediction));
        state_pub.publish(state_estimate);
        last_accel_x = accel_x;

        rate.sleep();
    }
    return 0;    
}
