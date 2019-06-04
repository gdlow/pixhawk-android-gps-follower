/**
 * Passes GPS and IMU data in ENU through a Kalman filter for better positioning accuracy
 * */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "GPSAccKalman.h"

ros::Publisher filtered_target_abs_pos_pub;
ros::Publisher filtered_target_wrtdrone_pos_pub;
ros::Publisher filtered_target_abs_vel_pub;

geometry_msgs::PointStamped relative_pos, filtered_abs_pos;
geometry_msgs::TwistStamped filtered_abs_vel;

double target_x, target_y, target_z, target_xVel, target_yVel, target_xAcc, target_yAcc;
double curr_time, prev_time, prev_x, prev_y;
GPSAccKalmanFilter_t* kf;

bool kf_initialised = false; // target_pos_callback
bool first_point_received = false; // target_pos_callback
bool target_vel_received = false; // target_pos_callback
bool target_pos_received = false; // target_pos_callback
bool target_acc_received = false; // target_acc_callback
bool drone_pos_received = false; // drone_pos_callback

double posDev = 1.49010849;
double accDev = 0.00004143908;

void target_pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    target_x = msg->point.x;
    target_y = msg->point.y;
    target_z = msg->point.z;
    target_pos_received = true;
    curr_time = ((double) ros::Time::now().sec) + ((double) (ros::Time::now().nsec / 1e09));
    if (first_point_received) {
        target_xVel = (target_x - prev_x) / (curr_time - prev_time);
        target_yVel = (target_y - prev_y) / (curr_time - prev_time);
        target_vel_received = true;
    } else {
        first_point_received = true;
    }
    prev_x = target_x;
    prev_y = target_y;
    prev_time = curr_time;
    // Ensures that kf only initialized after 2 points -> target_vel_received
    if (!kf_initialised && target_vel_received) {
        kf = GPSAccKalmanAlloc(target_x, target_y, target_xVel, target_yVel, accDev, posDev, curr_time);
        kf_initialised = true;
    } else {
        // Ensures that kf only updates after initialization + target_vel_received
        if (target_vel_received) {
            GPSAccKalmanUpdate(kf, curr_time, target_x, target_y, target_xVel, target_yVel, posDev);
        }
    }
}

void target_acc_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    target_xAcc = msg->linear_acceleration.x;
    target_yAcc = msg->linear_acceleration.y;
    double imu_time = ((double) ros::Time::now().sec) + ((double) (ros::Time::now().nsec / 1e09));
    target_acc_received = true;
    if (kf_initialised) {
        // Update Kalman Filter
        GPSAccKalmanPredict(kf, imu_time, target_xAcc, target_yAcc);
        // Retrieve abs position and velocities from kf
        filtered_abs_pos.header.stamp = ros::Time::now();
        filtered_abs_pos.header.frame_id = "enu";
        filtered_abs_pos.point.x = GPSAccKalmanGetX(kf);
        filtered_abs_pos.point.y = GPSAccKalmanGetY(kf);
        filtered_abs_pos.point.z = target_z;
        filtered_abs_vel.header.stamp = ros::Time::now();
        filtered_abs_vel.header.frame_id = "enu";
        filtered_abs_vel.twist.linear.x = GPSAccKalmanGetXVel(kf);
        filtered_abs_vel.twist.linear.y = GPSAccKalmanGetYVel(kf);
        // Publish
        filtered_target_abs_pos_pub.publish(filtered_abs_pos);
        filtered_target_abs_vel_pub.publish(filtered_abs_vel);
    }
}

void drone_pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Calculate relative position
    if (kf_initialised) {
        relative_pos.header = msg->header;
        relative_pos.header.frame_id = "enu";
        relative_pos.point.x = GPSAccKalmanGetX(kf) - msg->point.x;
        relative_pos.point.y = GPSAccKalmanGetY(kf) - msg->point.y;
        relative_pos.point.z = target_z - msg->point.z;
    } else {
        relative_pos.header = msg->header;
        relative_pos.header.frame_id = "enu";
        relative_pos.point.x = target_x - msg->point.x;
        relative_pos.point.y = target_y - msg->point.y;
        relative_pos.point.z = target_z - msg->point.z;
    }
    drone_pos_received = true;
    // Publish
    filtered_target_wrtdrone_pos_pub.publish(relative_pos);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kf_node");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);

    // subscribers
    ros::Subscriber target_pos_sub = nh.subscribe<geometry_msgs::PointStamped>("/target_position", 1, &target_pos_callback);
    ros::Subscriber target_acc_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, &target_acc_callback);
    ros::Subscriber drone_pos_sub = nh.subscribe<geometry_msgs::PointStamped>("/drone_position", 10, &drone_pos_callback);

    // publishers (all at 10Hz)
    filtered_target_abs_pos_pub = nh.advertise<geometry_msgs::PointStamped>("/filtered_target_abs_position", 10);
    filtered_target_wrtdrone_pos_pub = nh.advertise<geometry_msgs::PointStamped>("/filtered_target_wrtdrone_position", 10);
    filtered_target_abs_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/filtered_target_abs_velocity", 10);

    // wait for position and IMU information
    while (ros::ok() && !target_pos_received) {
        ROS_INFO_ONCE("Waiting for Target position...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Target position received");
    // TODO: Commented for testing
//    while (ros::ok() && !drone_pos_received) {
//        ROS_INFO_ONCE("Waiting for Drone position...");
//        ros::spinOnce();
//        rate.sleep();
//    }
//    ROS_INFO("Drone position received");
    while (ros::ok() && !target_acc_received) {
        ROS_INFO_ONCE("Waiting for Target IMU...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Target IMU received");

    ros::spin();
}