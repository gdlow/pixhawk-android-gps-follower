/**
 * @file cloud_echoer_node.cpp
 * @brief Echoes GPS, IMU and MagneticField data of a rosbridge topic to a conventional rostopic
 */

#include "rosbridge_ws_client.hpp"
#include <future>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

RosbridgeWsClient rbc("gdpdrone.serveo.net:80");

sensor_msgs::NavSatFix target_gps;
sensor_msgs::Imu target_imu;
sensor_msgs::MagneticField target_mag;

void remote_gps_cb(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::Message> message)
{
    rapidjson::Document document;
    std::string messagebuf = message->string();
    document.Parse(messagebuf.c_str());
    target_gps.latitude = document["msg"]["latitude"].GetDouble();
    target_gps.longitude = document["msg"]["longitude"].GetDouble();
    target_gps.altitude = document["msg"]["altitude"].GetDouble();
}

void remote_imu_cb(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::Message> message)
{
    rapidjson::Document document;
    std::string messagebuf = message->string();
    document.Parse(messagebuf.c_str());
    // Parse orientation
    target_imu.orientation.x = document["msg"]["orientation"]["x"].GetDouble();
    target_imu.orientation.y = document["msg"]["orientation"]["y"].GetDouble();
    target_imu.orientation.z = document["msg"]["orientation"]["z"].GetDouble();
    target_imu.orientation.w = document["msg"]["orientation"]["w"].GetDouble();
    // Parse angular_velocity
    target_imu.angular_velocity.x = document["msg"]["angular_velocity"]["x"].GetDouble();
    target_imu.angular_velocity.y = document["msg"]["angular_velocity"]["y"].GetDouble();
    target_imu.angular_velocity.z = document["msg"]["angular_velocity"]["z"].GetDouble();
    // Parse linear_acceleration
    target_imu.linear_acceleration.x = document["msg"]["linear_acceleration"]["x"].GetDouble();
    target_imu.linear_acceleration.y = document["msg"]["linear_acceleration"]["y"].GetDouble();
    target_imu.linear_acceleration.z = document["msg"]["linear_acceleration"]["z"].GetDouble();
}

void remote_mag_cb(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::Message> message)
{
    rapidjson::Document document;
    std::string messagebuf = message->string();
    document.Parse(messagebuf.c_str());
    // Parse magnetic_field
    target_mag.magnetic_field.x = document["msg"]["magnetic_field"]["x"].GetDouble();
    target_mag.magnetic_field.y = document["msg"]["magnetic_field"]["y"].GetDouble();
    target_mag.magnetic_field.z = document["msg"]["magnetic_field"]["z"].GetDouble();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_echoer_node");
    ros::NodeHandle nh;

    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/android/fix", 2);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/android/imu", 10);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("/android/magnetic_field", 10);

    rbc.addClient("gps_subscriber");
    rbc.subscribe("gps_subscriber", "/remote_gps", remote_gps_cb);
    rbc.addClient("imu_subscriber");
    rbc.subscribe("imu_subscriber", "/remote_imu", remote_imu_cb);
    rbc.addClient("mag_subscriber");
    rbc.subscribe("mag_subscriber", "/remote_mag", remote_mag_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    while(ros::ok()){
        target_gps.header.stamp = ros::Time::now();
        target_imu.header.stamp = ros::Time::now();
        target_mag.header.stamp = ros::Time::now();

        gps_pub.publish(target_gps);
        imu_pub.publish(target_imu);
        mag_pub.publish(target_mag);

        ros::spinOnce();
        // Verify that GPS data is coming through
        ROS_INFO_THROTTLE(1, "Target at latitude %f", target_gps.latitude);
        ROS_INFO_THROTTLE(1, "Target at longitude %f", target_gps.longitude);
        rate.sleep();
    }

    return 0;
}
