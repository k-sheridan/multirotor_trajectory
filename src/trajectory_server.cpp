#include "ros/ros.h"
#include <pauvsi_trajectory/trajectoryGeneration.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_trajectory", ros::init_options::AnonymousName); // initializes with a randomish name

	ros::NodeHandle nh;

	ros::spinOnce();
	return 0;
}
