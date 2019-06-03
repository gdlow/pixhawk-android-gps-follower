/**
 * Adapted from geodetic_utils library for only GPS -> ENU data
 * Requires: set_gps_reference_node to be run
 * */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <follower/geodetic_conv.hpp>
#include <std_msgs/Float64.h>

geodetic_converter::GeodeticConverter g_geodetic_converter;

ros::Publisher target_pos_pub;
ros::Publisher drone_pos_pub;

// ros parameters
bool g_trust_gps;
std::string g_frame_id;

void target_gps_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
        ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
        return;
    }

    if (!g_geodetic_converter.isInitialised()) {
        ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
        return;
    }

    double x, y, z;
    g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);

    // Fill up position message
    geometry_msgs::PointStampedPtr position_msg(
            new geometry_msgs::PointStamped);
    position_msg->header = msg->header;
    position_msg->header.frame_id = g_frame_id;
    position_msg->point.x = x;
    position_msg->point.y = y;
    position_msg->point.z = z;

    target_pos_pub.publish(position_msg);
}

void drone_gps_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
        ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
        return;
    }

    if (!g_geodetic_converter.isInitialised()) {
        ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
        return;
    }

    double x, y, z;
    g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);

    // Fill up position message
    geometry_msgs::PointStampedPtr position_msg(
            new geometry_msgs::PointStamped);
    position_msg->header = msg->header;
    position_msg->header.frame_id = g_frame_id;
    position_msg->point.x = x;
    position_msg->point.y = y;
    position_msg->point.z = z;

    drone_pos_pub.publish(position_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_conversion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Specify whether covariances should be set manually or from GPS
  ros::param::param("~trust_gps", g_trust_gps, false);

  // Get manual parameters
  ros::param::param<std::string>("~frame_id",
                                 g_frame_id, "world");

  // Wait until GPS reference parameters are initialized.
  double latitude, longitude, altitude;
  do {
    ROS_INFO("Waiting for GPS reference parameters...");
    if (nh.getParam("/gps_ref_latitude", latitude) &&
        nh.getParam("/gps_ref_longitude", longitude) &&
        nh.getParam("/gps_ref_altitude", altitude)) {
      g_geodetic_converter.initialiseReference(latitude, longitude, altitude);
    } else {
      ROS_INFO(
          "GPS reference not ready yet, use set_gps_reference_node to set it");
      ros::Duration(0.2).sleep(); // sleep for 0.2s
    }
  } while (!g_geodetic_converter.isInitialised());

  // Show target origin reference point
  double initial_latitude, initial_longitude, initial_altitude;
  g_geodetic_converter.getReference(&initial_latitude, &initial_longitude,
                                    &initial_altitude);
  ROS_INFO("GPS origin initialized correctly %f, %f, %f", initial_latitude,
           initial_longitude, initial_altitude);

  // Initialize target pos publisher
  target_pos_pub =
      nh.advertise<geometry_msgs::PointStamped>("/target_position", 1);

  // Initialize drone pos publisher
  drone_pos_pub =
      nh.advertise<geometry_msgs::PointStamped>("/drone_position", 10);

  // Subscribe to GPS Fixes and convert in callback
  ros::Subscriber target_sub = nh.subscribe<sensor_msgs::NavSatFix>("/android/fix", 1, &target_gps_callback);
  ros::Subscriber drone_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &drone_gps_callback);
  ros::spin();
}
