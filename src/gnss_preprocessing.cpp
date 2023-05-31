#include <ros/ros.h>

//#include "gnss_preprocessing_component.hpp"
#include "gnss_preprocessing_core.hpp"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "gnss_preprocessing");
    ros::NodeHandle n;

    double lat0 = 36.08263197663968;
    double lon0 = 140.07660776463374;
    double hig0 = 0.0;

    //GnssPreprocessingComponent gnss_preprocessing_component(n);

    GnssPreprocessingCore gnss_preprocessing_core();

	while(ros::ok()){
        //std::cout << "ROS_Interface Start!" << std::endl;
        ros::spinOnce();
	}

    return 0;
}