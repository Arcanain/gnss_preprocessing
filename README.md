# gnss_preprocessing

## FlowChart

```mermaid
flowchart TD
    A[Start] --> B[Initialize ROS Node: gnss_preprocessing]
    B --> C[Create GnssPreprocessingCore Object with Initial GPS Coordinates]
    C --> D[Set Up ROS Publishers and Subscriber]
    D --> E[Main ROS Loop]
    E --> F[Check if ROS is OK]
    F -->|Yes| G[Wait for GNSS Data Callback]
    G --> E
    F -->|No| H[End]

    subgraph GnssPreprocessingCore
        I[Constructor: Initialize Variables and Set Up ROS Publishers and Subscriber]
        J[gnssCallback: Process GNSS Data]
        K[Convert Lat/Lon/Alt to ECEF Coordinates]
        L[Convert ECEF to ENU Coordinates]
        M[Publish GNSS Pose and Path]
        N[Transform Broadcast: /odom to /base_link]
    end

    I --> J
    J --> K
    K --> L
    L --> M
    M --> N

```

## UML

```mermaid
classDiagram
    class GnssPreprocessingCore {
        +double pi
        +double a
        +double ONE_F
        +double E2
        -ros::NodeHandle nh
        -double lat0
        -double lon0
        -double hig0
        -ros::Publisher gnss_pose_pub
        -ros::Publisher gnss_path_pub
        -nav_msgs::Path gnss_path
        -ros::Subscriber gnss_sub
        -tf::TransformBroadcaster odom_to_baselink_broadcaster
        -geometry_msgs::TransformStamped odom_to_baselink_trans
        +GnssPreprocessingCore(double, double, double)
        -~GnssPreprocessingCore()
        -void gnssCallback(const sensor_msgs::NavSatFixConstPtr&)
        -double deg2rad(double)
        -double rad2deg(double)
        -Eigen::Vector3d blh2ecef(double, double, double)
        -Eigen::Vector3d ecef2blh(double, double, double)
        -Eigen::Vector3d ecef2enu(Eigen::Vector3d, Eigen::Vector3d)
        -Eigen::Matrix3d rotx(double)
        -Eigen::Matrix3d roty(double)
        -Eigen::Matrix3d rotz(double)
        -int blh2enu(double, double, double, double, double*, double*)
        -double constrain(double, double, double)
    }

    GnssPreprocessingCore --|> ros::NodeHandle: Uses
    GnssPreprocessingCore --|> ros::Publisher: Uses
    GnssPreprocessingCore --|> nav_msgs::Path: Uses
    GnssPreprocessingCore --|> ros::Subscriber: Uses
    GnssPreprocessingCore --|> tf::TransformBroadcaster: Uses
    GnssPreprocessingCore --|> geometry_msgs::TransformStamped: Uses
    GnssPreprocessingCore --|> Eigen: Uses
    GnssPreprocessingCore --|> sensor_msgs::NavSatFixConstPtr: Uses
```
