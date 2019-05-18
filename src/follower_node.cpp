/**
 * @file follower_node.cpp
 * @brief Offboard control example node to follow an android GPS waypoint
 */

#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::GlobalPositionTarget target_pos;
void get_target_pos(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	target_pos.latitude = msg->latitude;
	target_pos.longitude = msg->longitude;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower_node");
    ros::NodeHandle nh;

    // Initialise sub/pub and service clients
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // Android target position
    ros::Subscriber target_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
    		("/android/fix", 10, get_target_pos);
    ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("mavros/setpoint_raw/global", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
    	target_pos_pub.publish(target_pos);
    	ROS_INFO("Publishing latitude: [%f]", target_pos.latitude);
    	ROS_INFO("Publishing longitude: [%f]", target_pos.longitude);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){

        target_pos_pub.publish(target_pos);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}