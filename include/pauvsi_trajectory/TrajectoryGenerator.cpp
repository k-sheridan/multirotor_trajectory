/*
 * TrajectoryGenerator.cpp
 *
 *  Created on: Nov 5, 2016
 *      Author: kevin
 */

#include "TrajectoryGenerator.h"

TrajectoryGenerator::TrajectoryGenerator() {
	// TODO Auto-generated constructor stub

}

TrajectoryGenerator::~TrajectoryGenerator() {
	// TODO Auto-generated destructor stub
}

Eigen::Matrix<double, 10, 10> TrajectoryGenerator::generatePolyMatrix(double tf)
{
	double tf_2 = tf*tf;
	double tf_3 = tf_2*tf;
	double tf_4 = tf_3*tf;
	double tf_5 = tf_4*tf;
	double tf_6 = tf_5*tf;
	double tf_7 = tf_6*tf;
	double tf_8 = tf_7*tf;
	double tf_9 = tf_8*tf;

	Eigen::Matrix<double, 10, 10> A;
	A <<  0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 2, 0, 0,
			0, 0, 0, 0, 0, 0, 6, 0, 0, 0,
			0, 0, 0, 0, 0, 24, 0, 0, 0, 0,
			tf_9,         tf_8,         tf_7,         tf_6,         tf_5,       tf_4,     tf_3,   tf_2, tf, 1,
			9*tf_8,       8*tf_7,       7*tf_6,       6*tf_5,       5*tf_4,     4*tf_3,   3*tf_2, 2*tf, 1,  0,
			72*tf_7,     56*tf_6,     42*tf_5,     30*tf_4,     20*tf_3,   12*tf_2, 6*tf, 2,    0,  0,
			504*tf_6,   336*tf_5,   210*tf_4,   120*tf_3,   60*tf_2, 24*tf, 6,      0,    0,  0,
			3024*tf_5, 1680*tf_4, 840*tf_3, 360*tf_2, 120*tf, 24, 0, 0, 0, 0;
	return A;
}

Polynomial TrajectoryGenerator::solvePoly(PolynomialConstraints constraints, double tf)
{
	Eigen::VectorXd b(10, 1);
	b << constraints.x0,constraints.dx0,constraints.ax0,constraints.jerk_x0,constraints.snap_x0,constraints.xf,constraints.dxf,constraints.axf,constraints.jerk_xf,constraints.snap_xf;

	return this->generatePolyMatrix(tf).inverse() * b;
}

/*
 * calculates all derivatives of trajectory segment and stores them
 */
EfficientTrajectorySegment TrajectoryGenerator::preComputeTrajectorySegment(TrajectorySegment pos)
{
	EfficientTrajectorySegment result;

	result.pos = pos;
	result.vel = polyDer(result.pos);
	result.accel = polyDer(result.vel);
	result.jerk = polyDer(result.accel);
	result.snap = polyDer(result.jerk);

	return result;
}

/*
 * tests if trajectory segment is physically feasible for the quadrotor to perform
 */
bool TrajectoryGenerator::testSegmentForFeasibilityFAST(TrajectorySegment seg, PhysicalCharacterisics physical)
{
	Polynomial ax = polyDer(polyDer(seg.x));
	Polynomial ay = polyDer(polyDer(seg.y));
	Polynomial az = polyDer(polyDer(seg.z));

	Polynomial snap_x = polyDer(polyDer(ax));
	Polynomial snap_y = polyDer(polyDer(ay));
	Polynomial snap_z = polyDer(polyDer(az));

	//first check if quad is in free fall
	if((polyMin(az, 0, seg.tf) + G) < physical.min_motor_thrust * 4){
		return false; // the quad would need to be inverted to fly this!
	}


}

/*
 * computes the motor forces required at time t
 * extremely theoretical but should work
 */
Eigen::Vector4d TrajectoryGenerator::calculateMotorForces(EfficientTrajectorySegment ts, PhysicalCharacterisics physical, double t)
{
	Eigen::Vector3d accel = polyVal(ts.accel, t); // acceleration at time t
	Eigen::Vector3d jerk = polyVal(ts.jerk, t); // jerk at time t
	Eigen::Vector3d snap = polyVal(ts.snap, t); // snap at time t

	// calculate the inertial frame force required at max acceleration
	Eigen::Vector3d F_inertial = physical.mass * accel;
	F_inertial(2) += physical.mass * G; // add the FORCE due to gravity

	double f_total = F_inertial.norm(); // the total force required at max acceleration

	Eigen::Vector3d F_inertial_bar = F_inertial / f_total; // should find the direction vector of f inertial

	//setup the F_body vectors
	Eigen::Vector3d F_body;
	F_body << 0, 0, f_total;

	Eigen::Vector3d F_body_bar;
	F_body_bar << 0, 0, 1;

	//calculate the F_dot_inertial
	Eigen::Vector3d F_dot_inertial = physical.mass * jerk;

	Eigen::Vector3d F_dot_inertial_bar = (F_dot_inertial / f_total) - (F_inertial * F_inertial.transpose() * F_dot_inertial) / pow(f_total, 3); // equation 3.20 in Cutler's paper


	//calculate the desired angular rate
	Eigen::Vector3d tempCross = F_inertial_bar.cross(F_dot_inertial_bar);

	Eigen::Vector3d omega_body;
	omega_body << tempCross(0), tempCross(1), 0; // project onto the x-y plane

	//calculate the second derivative of inertial frame force unit vector
	Eigen::Vector3d F_dot_dot_inertial = physical.mass * snap;

	Eigen::Vector3d F_dot_dot_inertial_bar = (F_dot_dot_inertial / f_total) -
			(2 * F_dot_inertial * F_inertial.transpose() * F_dot_inertial   +   F_inertial * F_dot_inertial.transpose() * F_dot_inertial    +
					F_inertial * F_inertial.transpose() * F_dot_dot_inertial) / pow(f_total, 3)  +
					(3 * F_inertial * F_inertial.transpose() * F_dot_inertial) / pow(f_total, 5);

	// calculate the angular acceleration now
	tempCross = F_inertial_bar.cross((F_dot_dot_inertial_bar - omega_body.cross(omega_body.cross(F_dot_inertial_bar))));

	Eigen::Vector3d omega_dot_body;
	omega_dot_body << tempCross(0), tempCross(1), 0; // this is the body angular acceleration about the x and y axis


}

