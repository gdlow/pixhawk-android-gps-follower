/**
 * @file cloud_echoer_node.cpp
 * @brief Echoes GPS data of a rosbridge topic to a conventional rostopic
 */

#include "rosbridge_ws_client.hpp"
#include <future>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

RosbridgeWsClient rbc("gdpdrone.serveo.net:80");

sensor_msgs::NavSatFix target_pos;

void subscriberCallback(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::Message> message)
{
    rapidjson::Document document;
    std::string messagebuf = message->string();
    document.Parse(messagebuf.c_str());
    target_pos.latitude = document["msg"]["latitude"].GetDouble();
    target_pos.longitude = document["msg"]["longitude"].GetDouble();
    target_pos.altitude = document["msg"]["altitude"].GetDouble();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_echoer_node");
    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<sensor_msgs::NavSatFix>("/android/fix", 10);

    rbc.addClient("topic_subscriber");
    rbc.subscribe("topic_subscriber", "/remote_gps", subscriberCallback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(2.0);

    while(ros::ok()){
        target_pos.header.stamp = ros::Time::now();
        publisher.publish(target_pos);

        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "Target at latitude %f", target_pos.latitude);
        ROS_INFO_THROTTLE(1, "Target at longitude %f", target_pos.longitude);
        rate.sleep();
    }

    return 0;
}
