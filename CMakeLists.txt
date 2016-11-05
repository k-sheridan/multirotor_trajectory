cmake_minimum_required(VERSION 2.8.3)
project(pauvsi_trajectory)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  cmake_modules
)

find_package(Eigen3 REQUIRED)

include_directories(
	include
	${Eigen_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
   DEPENDS eigen 
)


