/**
 * @file cloud_follower_node.cpp
 * @brief Offboard control example node to follow an android GPS waypoint, advertised over rosbridge
 * Fixed height travel (needs to be edited for obstacle avoidance)
 */

#include "rosbridge_ws_client.hpp"
#include <future>
#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>

RosbridgeWsClient rbc("gdp.serveo.net:80");
mavros_msgs::State current_state;
mavros_msgs::GlobalPositionTarget target_pos;
bool global_position_received = false;

// callback functions
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Fixed altitude throughout
void get_target_alt(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position_received = true;
    target_pos.altitude = msg->altitude;
}

// TODO: Add coordinate_frame: either 5 or 6
void get_target_pos(std::shared_ptr<WsClient::Connection> /*connection*/, std::shared_ptr<WsClient::Message> message)
{
  rapidjson::Document document;
  std::string messagebuf = message->string();
  // std::cout << messagebuf << std::endl; // for debugging purposes
  document.Parse(messagebuf.c_str());
  target_pos.latitude = document["msg"]["latitude"].GetDouble();
  target_pos.longitude = document["msg"]["longitude"].GetDouble();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_follower_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber curr_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/mavros/global_position/raw/fix", 1, get_target_alt);
    ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("/mavros/setpoint_position/global", 10);
    // std::string name(argv[1]);
    // std::string site(".serveo.net:80");
    // std::string conn = name + site;
    // RosbridgeWsClient rbc(conn);
    rbc.addClient("topic_subscriber");
    rbc.subscribe("topic_subscriber", "/remote_gps", get_target_pos);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // wait for position information
    while (ros::ok() && !global_position_received) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");

    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        target_pos.header.stamp = ros::Time::now();
    	target_pos_pub.publish(target_pos);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        target_pos.header.stamp = ros::Time::now();
        target_pos_pub.publish(target_pos);

        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "Target at latitude %f", target_pos.latitude);
        ROS_INFO_THROTTLE(1, "Target at longitude %f", target_pos.longitude);
        rate.sleep();
    }

    return 0;
}