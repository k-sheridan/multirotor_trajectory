#include "ros/ros.h"
#include <pauvsi_trajectory/trajectoryGeneration.h>

bool generateTrajectory(pauvsi_trajectory::trajectoryGeneration::Request &req, pauvsi_trajectory::trajectoryGeneration::Response &res)
{
	//ROS_INFO("Starting trajectory generation from <%f, %f, %f> to <%f, %f, %f>", req.quadPosition.x, req.quadPosition.y, req.quadPosition.z, req.goalPosition.x, req.goalPosition.y, req.goalPosition.z);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_trajectory", ros::init_options::AnonymousName); // initializes with a randomish name

	ros::NodeHandle nh;

	ros::ServiceServer service = nh.advertiseService("generate_trajectory", generateTrajectory);

	ros::spinOnce();
	return 0;
}
