cmake_minimum_required(VERSION 3.0.2)
project(gnss_preprocessing)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gnss_preprocessing
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../../devel/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(gnss_preprocessing src/gnss_preprocessing.cpp)
target_link_libraries(gnss_preprocessing ${catkin_LIBRARIES})