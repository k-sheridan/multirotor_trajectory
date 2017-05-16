/*
 * Types.h
 *
 *  Created on: May 11, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_
#define PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_


struct PolynomialConstraints{
	double x0, xf;
	double dx0, dxf;
	double ax0, axf; // 2nd der
	double jerk_x0, jerk_xf; // 3rd der
	double snap_x0, snap_xf; // 4th der
};

struct TrajectoryConstraints{
	PolynomialConstraints const_x, const_y, const_z;
};

struct Point{
	double x, y, z;
	Point(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}

	Point(){
		x = 0;
		y = 0;
		z = 0;
	}
};

struct AdvancedWaypointConstraint{
	Point pos;
	Point vel;
	Point accel;
	Point jerk;
	Point snap;
	double t;
};

struct BasicWaypointConstraint{
	Point pos;
	double t;

	BasicWaypointConstraint(Point position, double time)
	{
		t = time;
		pos = position;
	}
};

struct DynamicTrajectoryConstraints{
	AdvancedWaypointConstraint start, end;
	std::vector<BasicWaypointConstraint> middle; // contains the middle points
};

typedef Eigen::VectorXd Polynomial; // highest order coeff first

struct TrajectorySegment{
	Polynomial x, y, z; //polynomial with respect to t
	double tf;
	double t0;

	TrajectorySegment()
	{
		tf = 0;
		t0 = 0;
	}
};

struct EfficientTrajectorySegment{
	TrajectorySegment pos, vel, accel, jerk, snap;
};

struct Trajectory{
	std::vector<TrajectorySegment> segments; // contains the segments of this trajectory
};

/*
 * desribes the physical constraints of the quadrotor
 */
struct PhysicalCharacterisics{
	double mass; // kg

	Eigen::Matrix3d J; // diagonal moment of inertia matrix - kg*m^2

	double max_motor_thrust; // N
	double min_motor_thrust; // N

	Eigen::Matrix4d torqueTransition; // matrix which maps motor forces to total force and moments
	Eigen::Matrix4d torqueTransition_inv; // maps total force and moments to motor forces
};


#endif /* PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_ */
