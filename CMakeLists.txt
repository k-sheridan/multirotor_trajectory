cmake_minimum_required(VERSION 2.8.3)
project(pauvsi_trajectory)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  cmake_modules
  message_generation
  dynamic_reconfigure
  tf
  gazebo_msgs
)

find_package(Eigen3 REQUIRED)

find_package(Boost)

include_directories(
	include
	${Eigen_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

# add the service files
add_service_files(
	FILES
	trajectoryGeneration.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	nav_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
   DEPENDS eigen 
)

add_library(polynomial include/pauvsi_trajectory/Polynomial.hpp)
set_target_properties(polynomial PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(polynomial ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_library(types include/pauvsi_trajectory/Types.h)
set_target_properties(types PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(types ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_library(trajGen include/pauvsi_trajectory/TrajectoryGenerator.cpp)
target_link_libraries(trajGen ${Eigen_LIBRARIES} ${catkin_LIBRARIES} polynomial types)

add_executable(trajectory_server src/trajectory_server.cpp)
target_link_libraries(trajectory_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES} trajGen)
add_dependencies(trajectory_server ${PROJECT_NAME}_generate_messages_cpp)

add_executable(unit_test test/unit_test.cpp)
target_link_libraries(unit_test ${catkin_LIBRARIES} ${Eigen_LIBRARIES} trajGen polynomial)

add_executable(virtual_flight_controller src/virtual-flight-controller.cpp)
target_link_libraries(virtual_flight_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES} trajGen polynomial)


