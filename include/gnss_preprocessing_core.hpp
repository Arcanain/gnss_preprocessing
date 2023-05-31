#ifndef __GNSS_PREPROCESSING_CORE_HPP__
#define __GNSS_PREPROCESSING_CORE_HPP__

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/NavSatFix.h"

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>

using namespace std;

class GnssPreprocessingCore
{
    public:
        //GnssPreprocessingCore();
        //GnssPreprocessingCore(ros::NodeHandle &n);
        //GnssPreprocessingCore(double lat, double lon, double hig);
        GnssPreprocessingCore(ros::NodeHandle &n, double lat, double lon, double hig);
        ~GnssPreprocessingCore();

        double pi = 3.1415926535898;
        double a = 6378137.0;
        double ONE_F = 298.257223563;
        double E2 = ((1.0/ONE_F)*(2-(1.0/ONE_F)));

        Eigen::Vector3d origin;
    private:
        ros::NodeHandle nh;

        // publisher

        // publish data
        nav_msgs::Path gnss_path;

        // subscriber
        ros::Subscriber gnss_sub;

        // callback function
        void gnssCallback(const sensor_msgs::NavSatFixConstPtr& gnss_msg);
        
        double deg2rad(double deg); // Convert degrees to radians
        double rad2deg(double rad); // Convert radians to degrees
        Eigen::Vector3d blh2ecef(double p_deg, double l_deg, double h);
        Eigen::Vector3d ecef2blh(double x, double y, double z);
        Eigen::Vector3d ecef2enu(Eigen::Vector3d dest, Eigen::Vector3d origin);
        Eigen::Matrix3d rotx(double theta);
        Eigen::Matrix3d roty(double theta);
        Eigen::Matrix3d rotz(double theta);
};

/***********************************************************************
 * Initialize 
 **********************************************************************/
GnssPreprocessingCore::GnssPreprocessingCore(ros::NodeHandle &n, double lat, double lon, double hig)
{
    std::cout << "GnssPreprocessingCore Start!" << std::endl;
    
    std::cout << lat << std::endl;
    std::cout << lon << std::endl;
    std::cout << hig << std::endl;

    // ROS Node
    nh = n;

    origin(0) = lat;
    origin(1) = lon;
    origin(2) = hig;

    // Publisher
    //gnss_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/gnss_pose", 10);
    //gnss_path_pub = nh.advertise<nav_msgs::Path>("/gnss_path", 10);

    // Subscriber
    gnss_sub = nh.subscribe("/fix", 10, &GnssPreprocessingCore::gnssCallback, this);

    // init gnss_path
    gnss_path.header.frame_id = "map";
    gnss_path.header.stamp = ros::Time::now();
    gnss_path.header.seq = 0;
}

GnssPreprocessingCore::~GnssPreprocessingCore()
{
    std::cout << "GnssPreprocessingCore Finish" << std::endl;
}

void GnssPreprocessingCore::gnssCallback(const sensor_msgs::NavSatFixConstPtr& gnss_msg)
{
    /*
    std::cout << gnss_msg->latitude << std::endl;
    std::cout << gnss_msg->longitude << std::endl;
    std::cout << gnss_msg->altitude << std::endl;
    */

    /*
    std::cout << origin(0) << std::endl;
    std::cout << origin(1) << std::endl;
    std::cout << origin(2) << std::endl;
    */

    // Convert lat/lon/height to ECEF
    Eigen::Vector3d ecef_origin = blh2ecef(origin(0), origin(1), origin(2));
    /*
    std::cout << ecef_origin(0) << std::endl;
    std::cout << ecef_origin(1) << std::endl;
    std::cout << ecef_origin(2) << std::endl;
    */
    Eigen::Vector3d ecef = blh2ecef(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);
    
    Eigen::Vector3d enu = ecef2enu(ecef, ecef_origin);
    /*
    std::cout << enu(0) << std::endl;
    std::cout << enu(1) << std::endl;
    std::cout << enu(2) << std::endl;
    */

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

Eigen::Vector3d GnssPreprocessingCore::blh2ecef(double p_deg, double l_deg, double h) {
    double p_rad = deg2rad(p_deg);
    double l_rad = deg2rad(l_deg);

    double NN = (a / sqrt(1.0 - (E2)*pow(sin(p_rad),2)));

    double x = (NN+h)*cos(p_rad)*cos(l_rad);
    double y = (NN+h)*cos(p_rad)*sin(l_rad);
    double z = (NN*(1-E2)+h)*sin(p_rad);

    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d GnssPreprocessingCore::ecef2blh(double x, double y, double z) {
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

Eigen::Vector3d GnssPreprocessingCore::ecef2enu(Eigen::Vector3d dest, Eigen::Vector3d origin) {
    Eigen::Vector3d blh = ecef2blh(origin(0), origin(1), origin(2));
    
    Eigen::Matrix3d rotzp1 = rotz(90.0);
    Eigen::Matrix3d rotyp = roty(90.0-blh(0));
    Eigen::Matrix3d rotzp2 = rotz(blh(1));

    Eigen::Matrix3d mat_conv1 = rotzp1*rotyp;
    Eigen::Matrix3d mat_conv2 = mat_conv1*rotzp2;

    Eigen::Vector3d mov = dest - origin;
    return mat_conv2*mov;
}

Eigen::Matrix3d GnssPreprocessingCore::rotx(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotxa;
    rotxa << 1, 0, 0,
             0, cos(rad), sin(rad),
             0, -sin(rad), cos(rad);
    return rotxa;
}

Eigen::Matrix3d GnssPreprocessingCore::roty(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotya;
    rotya << cos(rad), 0, -sin(rad),
             0, 1, 0,
             sin(rad), 0, cos(rad);
    return rotya;
}

Eigen::Matrix3d GnssPreprocessingCore::rotz(double theta) {
    double rad = deg2rad(theta);

    Eigen::Matrix3d rotza;
    rotza << cos(rad), sin(rad), 0,
             -sin(rad), cos(rad), 0,
             0, 0, 1;
    return rotza;
}

// Convert degrees to radians
double GnssPreprocessingCore::deg2rad(double deg) {
    return deg * pi / 180.0;
}

// Convert radians to degrees
double GnssPreprocessingCore::rad2deg(double rad) {
    return rad * 180.0 / pi;
}

#endif  // __GNSS_PREPROCESSING_CORE_HPP__