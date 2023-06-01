#ifndef __GNSS_PREPROCESSING_COMPONENT_HPP__
#define __GNSS_PREPROCESSING_COMPONENT_HPP__

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

class GnssPreprocessingComponent
{
    public:
        GnssPreprocessingComponent(double lat, double lon, double hig);
        //GnssPreprocessingComponent(ros::NodeHandle &n, double lat, double lon, double hig);
        ~GnssPreprocessingComponent();

        double pi = 3.1415926535898;
        double a = 6378137.0;
        double ONE_F = 298.257223563;
        double E2 = ((1.0/ONE_F)*(2-(1.0/ONE_F)));
    private:
        // ros node handle
        ros::NodeHandle nh;

        // local variable
        double lat0;
        double lon0;
        double hig0;

        // publisher
        ros::Publisher gnss_pose_pub;
        ros::Publisher gnss_path_pub;

        // publish data
        nav_msgs::Path gnss_path;

        // subscriber
        ros::Subscriber gnss_sub;

        // tf publish
        tf::TransformBroadcaster odom_to_baselink_broadcaster;
        geometry_msgs::TransformStamped odom_to_baselink_trans;

        // callback function
        void gnssCallback(const sensor_msgs::NavSatFixConstPtr& gnss_msg);
        
        // convert (lat, lon, alt) to (x, y, z)
        double deg2rad(double deg); // Convert degrees to radians
        double rad2deg(double rad); // Convert radians to degrees
        Eigen::Vector3d blh2ecef(double p_deg, double l_deg, double h);
        Eigen::Vector3d ecef2blh(double x, double y, double z);
        Eigen::Vector3d ecef2enu(Eigen::Vector3d dest, Eigen::Vector3d origin);
        Eigen::Matrix3d rotx(double theta);
        Eigen::Matrix3d roty(double theta);
        Eigen::Matrix3d rotz(double theta);

        // different version convert (lat, lon, alt) to (x, y, z)
        int blh2enu(double lat0, double lon0, double lat, double lon, double *x, double *y);
        double constrain(double val, double min, double max);
};

#endif  // __GNSS_PREPROCESSING_COMPONENT_HPP__