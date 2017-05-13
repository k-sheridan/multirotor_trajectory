/*
 * unit_test.cpp
 *
 *  Created on: May 13, 2017
 *      Author: kevin
 */
#include "ros/ros.h"

#include "../include/pauvsi_trajectory/Polynomial.hpp"
#include "../include/pauvsi_trajectory/TrajectoryGenerator.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_trajectory_test", ros::init_options::AnonymousName); // initializes with a randomish name

	ros::NodeHandle nh;

	TrajectoryGenerator trajGen = TrajectoryGenerator();

	Polynomial p(3, 1);
	p << 2, 1, -1;
	Polynomial result(2, 1);
	result << 4, 1;
	ROS_INFO_STREAM(p.transpose() << " => " << polyDer(p).transpose());
	ROS_ASSERT(result == polyDer(p));


	Polynomial p2(4, 1);
	p2 << -6, 5, -4, 7;
	Polynomial result2(3, 1);
	result2 << -18, 10, -4;
	ROS_INFO_STREAM(p2.transpose() << " => " << polyDer(p2).transpose());
	ROS_ASSERT(result2 == polyDer(p2));

	Polynomial p3(4, 1);
	p3 << 0.5, -10, 10, 1;

	ROS_INFO_STREAM(p3.transpose() << " => " << polyDer(p3).transpose());

	ROS_INFO_STREAM("max val over interval 0 to 20 of " << p3.transpose() << " is " << polyVal(p3, polyMaxTime(p3, 0, 20)) << " at t=" << polyMaxTime(p3, 0, 20));


	Polynomial x(3, 1);
	Polynomial y(3, 1);
	Polynomial z(3, 1);
	x << -1, 2, 3;
	y << -2, 3, 4;
	z << -3, 4, 5;

	double t = 0;

	ROS_INFO_STREAM("max vec over 0-2: " << polyVectorMax(x, y, z, 0, 2, t));
	ROS_INFO_STREAM(" at t = " << t );
	ROS_INFO_STREAM("max vec over 0-5: " << polyVectorMax(x, y, z, 0, 5, t));
	ROS_INFO_STREAM(" at t = " << t );
	ROS_ASSERT(fabs(t - 5) < 0.1);





	return 0;
}


