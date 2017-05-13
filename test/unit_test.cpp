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

	//test poly sol
	PolynomialConstraints pc;
	pc.x0 = -10;
	pc.dx0 = 1;
	pc.ax0 = 4;
	pc.jerk_x0 = 0;
	pc.snap_x0 = 0;

	pc.xf = 0;
	pc.dxf = 9;
	pc.axf = 4;
	pc.jerk_xf = 0;
	pc.snap_xf = 0;

	Polynomial polyRes(10, 1);
	polyRes << 0, 0, 0, 0, 0, 0, 0, 2, 1, -10;

	ROS_INFO_STREAM("expect poly " << polyRes.transpose() << " got " << trajGen.solvePoly(pc, 2).transpose());


	Polynomial solution = trajGen.solvePoly(pc, 2);

	ROS_INFO_STREAM("t = 0: " << polyVal(solution, 0) << " t = 2: " << polyVal(solution, 2));
	ROS_INFO_STREAM("t = 0: " << polyVal(polyDer(solution), 0) << " t = 2: " << polyVal(polyDer(solution), 2));
	ROS_INFO_STREAM("t = 0: " << polyVal(polyDer(polyDer(solution)), 0) << " t = 2: " << polyVal(polyDer(polyDer(solution)), 2));

	Eigen::VectorXd b(10, 1);
	b << pc.x0,pc.dx0,pc.ax0,pc.jerk_x0,pc.snap_x0,pc.xf,pc.dxf,pc.axf,pc.jerk_xf,pc.snap_xf;

	ROS_ASSERT((trajGen.generatePolyMatrix(2) * solution - b).norm() < 0.0001);




	return 0;
}


