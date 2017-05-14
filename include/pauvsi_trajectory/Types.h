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

typedef Eigen::VectorXd Polynomial; // highest order coeff first

struct TrajectorySegment{
	Polynomial x, y, z; //polynomial with respect to t
	double tf;
	//t0 = 0
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
};


#endif /* PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_ */
