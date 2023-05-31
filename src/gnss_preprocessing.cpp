#include <ros/ros.h>

//#include "gnss_preprocessing_component.hpp"
#include "gnss_preprocessing_core.hpp"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "gnss_preprocessing");
    ros::NodeHandle n;

    // for utbm_robocar_dataset_20190131_noimage.bag
    // Init gps position(latitude, longitude)
    double lat0 = 47.5115140833;
    double lon0 = 6.79310693333;
    double hig0 = 0.0;

    /*
    double lat0 = 36.08263197663968;
    double lon0 = 140.07660776463374;
    double hig0 = 0.0;
    */

    //GnssPreprocessingComponent gnss_preprocessing_component(n);

    //GnssPreprocessingCore gnss_preprocessing_core(n);

    GnssPreprocessingCore gnss_preprocessing_core(n, lat0, lon0, hig0);

	while(ros::ok()){
        //std::cout << "ROS_Interface Start!" << std::endl;
        ros::spinOnce();
	}

    return 0;
}