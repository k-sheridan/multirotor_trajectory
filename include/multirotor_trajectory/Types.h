/*
 * Types.h
 *
 *  Created on: May 11, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_
#define PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_

#include <ros/ros.h>

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

	Point(Eigen::Vector3d vec)
	{
		x = vec(0);
		y = vec(1);
		z = vec(2);
	}

	Eigen::Vector3d toEigen()
	{
		Eigen::Vector3d vec;
		vec << x, y, z;
		return vec;
	}
};

struct GeometricConstraint{
	enum Type {
		Z_PLANE_MIN,
		Z_PLANE_MAX
	};

	Type type;

	double z_min;
	double z_max;
	double radius;

	GeometricConstraint(){}

	GeometricConstraint(Type _type, double val)
	{
		type = _type;
		switch(type)
		{
		case Z_PLANE_MIN:
			z_min = val;
			break;
		case Z_PLANE_MAX:
			z_max = val;
			break;
		}
	}

};

struct AdvancedWaypointConstraint{
	Point pos;
	Point vel;
	Point accel;
	Point jerk;
	Point snap;
	double t;

	std::vector<GeometricConstraint> geoConstraint; // constraint between this and the next waypoint
};

struct BasicWaypointConstraint{
	Point pos;
	double t;

	BasicWaypointConstraint(Point position, double time)
	{
		t = time;
		pos = position;
	}

	std::vector<GeometricConstraint> geoConstraint; // constraint between this and the next waypoint
};

struct DynamicTrajectoryConstraints{
	AdvancedWaypointConstraint start, end;
	std::vector<BasicWaypointConstraint> middle; // contains the middle points

	Eigen::VectorXd getTimes()
	{
		Eigen::VectorXd times(this->middle.size()+2);

		times(0) = this->start.t;
		times(times.size()-1) = this->end.t;
		for(int i = 0; i < this->middle.size(); i++)
		{
			times(1+i) = this->middle[i].t;
		}
		return times;
	}

	DynamicTrajectoryConstraints* assignTimes(Eigen::VectorXd times)
	{
		ROS_ASSERT((times.size() - 2) == this->middle.size());

		this->start.t = times(0);
		this->end.t = times(times.size()-1);

		for(int i = 0; i < this->middle.size(); i++)
		{
			this->middle[i].t = times(1+i);
		}

		return this;

	}

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
