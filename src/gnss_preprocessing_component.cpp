#include "ros/ros.h"
#include "gnss_preprocessing_component.hpp"

#include <iostream>
#include <cmath>

using namespace std;

/***********************************************************************
 * Initialize 
 **********************************************************************/
GnssPreprocessingComponent::GnssPreprocessingComponent(ros::NodeHandle &n)
{
    std::cout << "GnssPreprocessingComponent Start!" << std::endl;
    
    // ROS Node
    nh = n;

    // Publisher
    //gnss_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_pose", 10);
    //gnss_path_pub = nh.advertise<nav_msgs::Path>("/gnss_path", 10);

    // Subscriber
    gnss_sub = nh.subscribe("/fix", 10, &GnssPreprocessingComponent::gnssCallback, this);

    // init gps_path
    gps_path.header.frame_id = "map";
    gps_path.header.stamp = ros::Time::now();
    gps_path.header.seq = 0;
}

GnssPreprocessingComponent::~GnssPreprocessingComponent()
{
    std::cout << "GnssPreprocessingComponent Finish" << std::endl;
}

Eigen::Vector3d GnssPreprocessingComponent::blh2ecef(double p_deg, double l_deg, double h) {
    double p_rad = deg2rad(p_deg);
    double l_rad = deg2rad(l_deg);

    double NN = (a / sqrt(1.0 - (E2)*pow(sin(p_rad),2)));

    double x = (NN+h)*cos(p_rad)*cos(l_rad);
    double y = (NN+h)*cos(p_rad)*sin(l_rad);
    double z = (NN*(1-E2)+h)*sin(p_rad);

    return Eigen::Vector3d(x, y, z);
}

void GnssPreprocessingComponent::gnssCallback(const sensor_msgs::NavSatFixConstPtr& gnss_msg)
{
    std::cout << "gnss callback" << std::endl;

    /*
    data_conversion_gps(gps_msg, gps_data);

    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.header.stamp = ros::Time::now();
    point.pose.position.x = gps_data.ned[0];
    point.pose.position.y = gps_data.ned[1];
    point.pose.position.z = 0.0;
    point.pose.orientation.w = 0;
    point.pose.orientation.x = 0;
    point.pose.orientation.y = 0;
    point.pose.orientation.z = 1.0;
    gps_path.poses.push_back(point);
    gps_pose_pub.publish(point);
    gps_path_pub.publish(gps_path);

    // /odom to /base_link transform broadcast
    odom_to_baselink_trans.header.stamp = ros::Time::now();
    odom_to_baselink_trans.header.frame_id = "odom";
    odom_to_baselink_trans.child_frame_id = "base_link";
    odom_to_baselink_trans.transform.translation.x = gps_data.ned[0];
    odom_to_baselink_trans.transform.translation.y = gps_data.ned[1];
    odom_to_baselink_trans.transform.translation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.x = 0.0;
    odom_to_baselink_trans.transform.rotation.y = 0.0;
    odom_to_baselink_trans.transform.rotation.z = 0.0;
    odom_to_baselink_trans.transform.rotation.w = 1.0;
    odom_to_baselink_broadcaster.sendTransform(odom_to_baselink_trans);
    */
}

Eigen::Vector3d GnssPreprocessingComponent::ecef2blh(double x, double y, double z) {
    double b = (a*(1.0 - 1.0/ONE_F));
    double ED2 = (E2*pow(a,2)/(pow(b,2)));
    double p = sqrt(pow(x,2) + pow(y,2));
    double si = atan2(z*a, p*b);

    double p_rad2 = atan2((z + ED2*b*pow(sin(si),3)),(p-E2*a*pow(cos(si),3)));
    double l_rad2 = atan2(y, x);

    double lat = rad2deg(p_rad2);
    double lon = rad2deg(l_rad2);

    double NN = (a / sqrt(1.0 - (E2)*pow(sin(p_rad2),2)));
    double hig = (p/cos(p_rad2)) - NN;

    return Eigen::Vector3d(lat, lon, hig);
}

Eigen::Vector3d GnssPreprocessingComponent::ecef2enu(Eigen::Vector3d dest, Eigen::Vector3d origin) {
    Eigen::Vector3d blh = ecef2blh(origin(0), origin(1), origin(2));
    
    Eigen::Matrix3d rotzp1 = rotz(90.0);
    Eigen::Matrix3d rotyp = roty(90.0-blh(0));
    Eigen::Matrix3d rotzp2 = rotz(blh(1));

    Eigen::Matrix3d mat_conv1 = rotzp1*rotyp;
    Eigen::Matrix3d mat_conv2 = mat_conv1*rotzp2;

    Eigen::Vector3d mov = dest - origin;
    return mat_conv2*mov;
}

Eigen::Matrix3d GnssPreprocessingComponent::rotx(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotxa;
    rotxa << 1, 0, 0,
             0, cos(rad), sin(rad),
             0, -sin(rad), cos(rad);
    return rotxa;
}

Eigen::Matrix3d GnssPreprocessingComponent::roty(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotya;
    rotya << cos(rad), 0, -sin(rad),
             0, 1, 0,
             sin(rad), 0, cos(rad);
    return rotya;
}

Eigen::Matrix3d GnssPreprocessingComponent::rotz(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotza;
    rotza << cos(rad), sin(rad), 0,
             -sin(rad), cos(rad), 0,
             0, 0, 1;
    return rotza;
}

// Convert degrees to radians
double GnssPreprocessingComponent::deg2rad(double deg) {
    return deg * pi / 180.0;
}

// Convert radians to degrees
double GnssPreprocessingComponent::rad2deg(double rad) {
    return rad * 180.0 / pi;
}