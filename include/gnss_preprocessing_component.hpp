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

class GnssPreprocessingComponent
{
    public:
        //GnssPreprocessingComponent();
        GnssPreprocessingComponent(ros::NodeHandle &n);
        //GnssPreprocessingComponent(double lat, double lon, double hig);
        //GnssPreprocessingComponent(ros::NodeHandle &n, double lat, double lon, double hig);
        ~GnssPreprocessingComponent();

        double pi = 3.1415926535898;
        double a = 6378137.0;
        double ONE_F = 298.257223563;
        double E2 = ((1.0/ONE_F)*(2-(1.0/ONE_F)));
    private:
        ros::NodeHandle nh;

        // gps callback
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

#endif  // __GNSS_PREPROCESSING_COMPONENT_HPP__