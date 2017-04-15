cmake_minimum_required(VERSION 2.8.3)
project(pauvsi_trajectory)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  cmake_modules
  message_generation
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

include_directories(
	include
	${Eigen_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
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

add_executable(trajectory_server src/trajectory_server.cpp)
target_link_libraries(trajectory_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(trajectory_server ${PROJECT_NAME}_generate_messages_cpp)


