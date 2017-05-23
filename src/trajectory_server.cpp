#include "ros/ros.h"
#include <pauvsi_trajectory/trajectoryGeneration.h>
#include <nav_msgs/Path.h>

#include "../include/pauvsi_trajectory/Config.h"
#include "../include/pauvsi_trajectory/Polynomial.hpp"
#include "../include/pauvsi_trajectory/Physics.h"

#include "../include/pauvsi_trajectory/TrajectoryGenerator.h"

#include <eigen3/Eigen/Geometry>

PhysicalCharacterisics phys;

bool generateTrajectory(pauvsi_trajectory::trajectoryGeneration::Request &req, pauvsi_trajectory::trajectoryGeneration::Response &res);

ros::Publisher path_pub;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pauvsi_trajectory", ros::init_options::AnonymousName); // initializes with a randomish name

	ros::NodeHandle nh;

	path_pub = nh.advertise<nav_msgs::Path>("currentTrajectory", 1);

	ros::ServiceServer service = nh.advertiseService("generate_trajectory", generateTrajectory);

	phys.J << J_MATRIX;
	phys.mass = MASS;
	phys.min_motor_thrust = MOTOR_FORCE_MIN;
	phys.max_motor_thrust = MOTOR_FORCE_MAX;
	phys.torqueTransition << TORQUE_TRANSITION;
	phys.torqueTransition_inv = phys.torqueTransition.inverse();

	ros::spin();
	return 0;
}

bool generateTrajectory(pauvsi_trajectory::trajectoryGeneration::Request &req, pauvsi_trajectory::trajectoryGeneration::Response &res)
{
	//ROS_INFO("Starting trajectory generation from <%f, %f, %f> to <%f, %f, %f>", req.quadPosition.x, req.quadPosition.y, req.quadPosition.z, req.goalPosition.x, req.goalPosition.y, req.goalPosition.z);
	TrajectoryGenerator tg;
	DynamicTrajectoryConstraints dc;

	dc.start.pos = Point(req.startPosition.x, req.startPosition.y, req.startPosition.z);
	dc.start.vel = Point(req.startVelocity.x, req.startVelocity.y, req.startVelocity.z);
	dc.start.accel = Point(req.startAcceleration.x, req.startAcceleration.y, req.startAcceleration.z);
	dc.start.jerk = Point(req.startJerk.x, req.startJerk.y, req.startJerk.z);
	dc.start.snap = Point(req.startSnap.x, req.startSnap.y, req.startSnap.z);

	dc.start.geoConstraint.push_back(GeometricConstraint(GeometricConstraint::PLANE_MAX, req.startMaxZ));
	dc.start.geoConstraint.push_back(GeometricConstraint(GeometricConstraint::PLANE_MIN, req.startMinZ));

	dc.end.pos = Point(req.goalPosition.x, req.goalPosition.y, req.goalPosition.z);
	dc.end.vel = Point(req.goalVelocity.x, req.goalVelocity.y, req.goalVelocity.z);
	dc.end.accel = Point(req.goalAcceleration.x, req.goalAcceleration.y, req.goalAcceleration.z);
	dc.end.jerk = Point(req.goalJerk.x, req.goalJerk.y, req.goalJerk.z);
	dc.end.snap = Point(req.goalSnap.x, req.goalSnap.y, req.goalSnap.z);

	for(int i = 0; i < (int)req.middle_size; i++)
	{
		dc.middle.push_back(BasicWaypointConstraint(Point(req.middle[i].x, req.middle[i].y, req.middle[i].z), 0));
		dc.middle[i].geoConstraint.push_back(GeometricConstraint(GeometricConstraint(GeometricConstraint::PLANE_MIN, req.middleGeometricConstraints[(2*i + 0)])));
		dc.middle[i].geoConstraint.push_back(GeometricConstraint(GeometricConstraint(GeometricConstraint::PLANE_MAX, req.middleGeometricConstraints[(2*i + 1)])));
	}

	//compute trajectory
	TrajectorySegment seg = tg.computeHighOrderMinimumTimeTrajectory(dc, phys);

	res.tf = seg.tf;

	res.trajectory.layout.data_offset = 0;
	res.trajectory.layout.dim[0].label = "number of polynomials";
	res.trajectory.layout.dim[0].stride = 3 * seg.x.size();
	res.trajectory.layout.dim[0].size = 3;

	res.trajectory.layout.dim[1].label = "coefficients";
	res.trajectory.layout.dim[1].stride = seg.x.size();
	res.trajectory.layout.dim[1].size = seg.x.size();

	res.trajectory.layout.dim[2].label = "none";
	res.trajectory.layout.dim[2].stride = 1;
	res.trajectory.layout.dim[2].size = 1;

	for(int i = 0; i < (int)seg.x.size(); i++)
	{
		res.trajectory.data.at(i) = seg.x[i];
	}
	for(int i = 0; i < (int)seg.y.size(); i++)
	{
		res.trajectory.data.at(seg.y.size()*1 + i) = seg.y[i];
	}
	for(int i = 0; i < (int)seg.z.size(); i++)
	{
		res.trajectory.data.at(seg.z.size()*2 + i) = seg.z[i];
	}

	return true;
}
