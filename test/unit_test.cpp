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

	ROS_INFO_STREAM("max vec over 0-2: " << polyVectorMaxFAST(x, y, z, 0, 2, t));
	ROS_INFO_STREAM(" at t = " << t );
	ROS_INFO_STREAM("max vec over 0-5: " << polyVectorMaxFAST(x, y, z, 0, 5, t));
	ROS_INFO_STREAM(" at t = " << t );
	ROS_ASSERT(fabs(t - 5) < 0.1);

	ROS_INFO_STREAM("min z time over 0-5: " << polyMinTime(z, 0, 5));

	//test poly sol
	PolynomialConstraints pc;
	pc.x0 = -10;
	pc.dx0 = 1;
	pc.ax0 = 4;
	pc.jerk_x0 = -9;
	pc.snap_x0 = 8;

	pc.xf = 0;
	pc.dxf = 9;
	pc.axf = 4;
	pc.jerk_xf = -1;
	pc.snap_xf = 1;

	Polynomial polyRes(10, 1);
	polyRes << 0, 0, 0, 0, 0, 0, 0, 2, 1, -10;

	ROS_INFO_STREAM("expect poly " << polyRes.transpose() << " got " << trajGen.solvePoly(pc, trajGen.generatePolyMatrix(2).lu().inverse()).transpose());


	Polynomial solution = trajGen.solvePoly(pc, trajGen.generatePolyMatrix(2).lu().inverse());

	ROS_INFO_STREAM("t = 0: " << polyVal(solution, 0) << " t = 2: " << polyVal(solution, 2));
	ROS_INFO_STREAM("t = 0: " << polyVal(polyDer(solution), 0) << " t = 2: " << polyVal(polyDer(solution), 2));
	ROS_INFO_STREAM("t = 0: " << polyVal(polyDer(polyDer(solution)), 0) << " t = 2: " << polyVal(polyDer(polyDer(solution)), 2));
	ROS_INFO_STREAM("t = 0: " << polyVal(polyDer(polyDer(polyDer(solution))), 0) << " t = 2: " << polyVal(polyDer(polyDer(polyDer(solution))), 2));
	ROS_INFO_STREAM("t = 0: " << polyVal(polyDer(polyDer(polyDer(polyDer(solution)))), 0) << " t = 2: " << polyVal(polyDer(polyDer(polyDer(polyDer(solution)))), 2));

	Eigen::VectorXd b(10, 1);
	b << pc.x0,pc.dx0,pc.ax0,pc.jerk_x0,pc.snap_x0,pc.xf,pc.dxf,pc.axf,pc.jerk_xf,pc.snap_xf;

	ROS_ASSERT((trajGen.generatePolyMatrix(2) * solution - b).norm() < 0.0001);

	//trajectory

	PhysicalCharacterisics phys;
	phys.mass = 5;
	phys.J << 0.75, 0, 0,
			0, 0.75, 0,
			0, 0, 0.75;

	phys.min_motor_thrust = 0.1;
	phys.max_motor_thrust = 22;

	phys.torqueTransition << 1, 1, 1, 1,
			0.25, 0.25, -0.25, -0.25,
			0.25, -0.25, -0.25, 0.25,
			-0.01, 0.01, -0.01, 0.01;

	phys.torqueTransition_inv = phys.torqueTransition.inverse();

	PolynomialConstraints c;
	c.x0 = 0;
	c.dx0 = 0;
	c.ax0 = 0;
	c.jerk_x0 = 0;
	c.snap_x0 = 0;

	c.xf = 2.5;
	c.dxf = 0;
	c.axf = 0;
	c.jerk_xf = 0;
	c.snap_xf = 0;

	TrajectoryConstraints tc;

	tc.const_z = c;
	c.xf = 1;
	tc.const_y = c;
	c.xf = 1;
	tc.const_x = c;


	TrajectorySegment ans = trajGen.computeMinimumTimeTrajectorySegment(tc, phys, 5.0);

	ROS_INFO_STREAM("time to fly: " << ans.tf);
	ROS_INFO_STREAM("X: " << ans.x.transpose() << "\nY: " << ans.y.transpose() << "\nZ: " << ans.z.transpose());

	EfficientTrajectorySegment eff_seg = trajGen.preComputeTrajectorySegment(ans);

	ROS_INFO_STREAM("motor force at start: " << trajGen.calculateMotorForces(eff_seg, phys, 0.0));
	ROS_INFO_STREAM("motor force at 1/4: " << trajGen.calculateMotorForces(eff_seg, phys, ans.tf / 4));
	ROS_INFO_STREAM("motor force at middle: " << trajGen.calculateMotorForces(eff_seg, phys, ans.tf / 2));
	ROS_INFO_STREAM("motor force at 3/4: " << trajGen.calculateMotorForces(eff_seg, phys, ans.tf *0.75));
	ROS_INFO_STREAM("motor force at end: " << trajGen.calculateMotorForces(eff_seg, phys, ans.tf));


	//DYNAMIC TRAJECTORY

	DynamicTrajectoryConstraints dc;

	dc.start.t = 0;
	dc.start.pos = Point(0, 0, 0);
	dc.start.vel = Point(0, 0, 0);
	dc.start.accel = Point(0, 0, 0);
	dc.start.jerk = Point(0, 0, 0);
	dc.start.snap = Point(0, 0, 0);

	dc.end.t = 6;
	dc.end.pos = Point(0, 0, 0.5);
	dc.end.vel = Point(0, 0, 0);
	dc.end.accel = Point(0, 0, 0);
	dc.end.jerk = Point(0, 0, 0);
	dc.end.snap = Point(0, 0, 0);

	Eigen::MatrixXd A = trajGen.generateDynamicPolyMatrix(dc);

	ROS_INFO_STREAM(trajGen.generatePolyMatrix(6));
	ROS_INFO_STREAM(A);

	ROS_ASSERT(A == trajGen.generatePolyMatrix(6));


	GeometricConstraint floor, ceil, obs;
	floor.type = GeometricConstraint::PLANE_MIN;
	floor.z_min = 0;
	obs.type = GeometricConstraint::PLANE_MIN;
	obs.z_min = 2;
	ceil.type = GeometricConstraint::PLANE_MAX;
	ceil.z_max = 3.0;

	dc.start.t = 0;
	dc.start.pos = Point(0, 0, 0.5);
	dc.start.vel = Point(0, 0, 0);
	dc.start.accel = Point(0, 0, 0);
	dc.start.jerk = Point(0, 0, 0);
	dc.start.snap = Point(0, 0, 0);
	dc.start.geoConstraint.push_back(ceil);
	dc.start.geoConstraint.push_back(floor);


	dc.end.t = 5;
	dc.end.pos = Point(9, 0, 0.5);
	dc.end.vel = Point(0, 0, 0);
	dc.end.accel = Point(0, 0, 0);
	dc.end.jerk = Point(0, 0, 0);
	dc.end.snap = Point(0, 0, 0);
	dc.end.geoConstraint.push_back(ceil);
	dc.end.geoConstraint.push_back(floor);


	dc.middle.push_back(BasicWaypointConstraint(Point(2, 0, 2.5), 2));
	dc.middle.back().geoConstraint.push_back(ceil);
	dc.middle.back().geoConstraint.push_back(obs);

	dc.middle.push_back(BasicWaypointConstraint(Point(4, 0, 2.5), 2.5));
	dc.middle.back().geoConstraint.push_back(ceil);
	dc.middle.back().geoConstraint.push_back(obs);


	dc.middle.push_back(BasicWaypointConstraint(Point(6, 0, 2.5), 2.5));
	dc.middle.back().geoConstraint.push_back(ceil);
	dc.middle.back().geoConstraint.push_back(floor);

	//dc.middle.push_back(BasicWaypointConstraint(Point(7, 0, 2.5), 4));

	A = trajGen.generateDynamicPolyMatrix(dc);

	TrajectorySegment trajectory = trajGen.computeHighOrderMinimumTimeTrajectory(dc, phys);


	ros::Publisher path_pub;
	path_pub = nh.advertise<nav_msgs::Path>("currentTrajectory", 1);

	nav_msgs::Path path = trajGen.generateTrajectorySegmentPath(trajectory);

	ROS_DEBUG_STREAM("path points: " << path.poses.size());

	ROS_DEBUG_STREAM(trajectory.t0 << " - " << trajectory.tf);

	ROS_INFO_STREAM("path length: " << trajGen.arcLengthTrajectoryBRUTE(trajectory));

	ROS_INFO_STREAM("feasible: " << trajGen.testSegmentForFeasibilityFAST(trajectory, phys));

	path_pub.publish(path);

	ros::Rate loop_rate(2);

	while(ros::ok())
	{
		path_pub.publish(path);
		loop_rate.sleep();
	}

	return 0;
}


