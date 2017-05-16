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

//DYNAMIC POLYNOMIALS
/*
 * ASSUMES: the start time is 0
 *
 * forms the polynomial matrix for a complex polynomial trajectory with variable midpoints
 */
Eigen::MatrixXd TrajectoryGenerator::generateDynamicPolyMatrix(DynamicTrajectoryConstraints constraints)
{
	int midPointCount = constraints.middle.size();
	int dim = 10 + midPointCount; // the two endpoints make 10 degrees and then each middle way point adds another

	ROS_DEBUG_STREAM("dynamic matrix size is " << dim);

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim, dim); // creates the dim X dim mat

	//setup the start constraint
	A(0, dim-1) = 1;
	A(1, dim-2) = 1;
	A(2, dim-3) = 2;
	A(3, dim-4) = 6;
	A(4, dim-5) = 24;

	for(int i = 0; i < midPointCount; i++)
	{
		double t_const = constraints.middle.at(i).t;
		double t_pow = 1;
		for(int j = 0; j < dim; j++)
		{
			ROS_DEBUG_STREAM("addressing: " << 5 + i << ", " << dim - 1 - j);
			A(5 + i, dim - j - 1) = t_pow;
			t_pow *= t_const; // increase power of t_pow
		}
	}

	Polynomial poly = Eigen::MatrixXd::Ones(dim, 1);
	double t_pow[dim];
	double t_const = constraints.end.t;
	ROS_ASSERT(dim > 2);
	t_pow[dim-1] = 1;
	for(int i = dim - 2; i >= 0; i--)
	{
		t_pow[i] = t_const * t_pow[i+1];
	}

	for(int i = 0; i < 5; i++)
	{
		for(int j = 0; j < dim; j++)
		{
			ROS_DEBUG_STREAM("addressing: " << 5 + i + midPointCount << ", " << j);
			if(i + j >= dim)
			{
				A(i + 5 + midPointCount, j) = 0;
			}
			else
			{
				A(i + 5 + midPointCount, j) = poly(j) * t_pow[i + j];
			}
		}

		poly = polyDer(poly);
	}

	return A;
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

Polynomial TrajectoryGenerator::solvePoly(PolynomialConstraints constraints, Eigen::Matrix<double, 10, 10> A_inv)
{
	Eigen::VectorXd b(10, 1);
	b << constraints.x0,constraints.dx0,constraints.ax0,constraints.jerk_x0,constraints.snap_x0,constraints.xf,constraints.dxf,constraints.axf,constraints.jerk_xf,constraints.snap_xf;

	return A_inv * b;
}

TrajectorySegment TrajectoryGenerator::solveSegment(TrajectoryConstraints constraints, double tf)
{
	TrajectorySegment seg;
	seg.tf = tf;

	Eigen::Matrix<double, 10, 10> A_inv = this->generatePolyMatrix(tf).lu().inverse();

	seg.x = this->solvePoly(constraints.const_x, A_inv);
	seg.y = this->solvePoly(constraints.const_y, A_inv);
	seg.z = this->solvePoly(constraints.const_z, A_inv);

	return seg;
}

/*
 * finds the minimum time feasible trajectory for these constraints
 */
TrajectorySegment TrajectoryGenerator::computeMinimumTimeTrajectorySegment(TrajectoryConstraints constraints, PhysicalCharacterisics physical, double tf_guess)
{
	TrajectorySegment seg;
	TrajectorySegment high_seg;

	double t_low = MIN_SEGMENT_TIME;
	double t_curr = tf_guess;
	double t_high = MAX_SEGMENT_TIME;

	for(int i = 0; i < TIME_OPTIM_ITER - 1; i++)
	{
		seg = solveSegment(constraints, t_curr); // solve to segment at curr
		if(testSegmentForFeasibilityFAST(seg, physical)) // if the segment is too slow
		{
			t_high = t_curr;
			t_curr = t_low + (t_high - t_low) / 2;
			high_seg = seg;
		}
		else
		{
			t_low = t_curr;
			t_curr = t_low + (t_high - t_low) / 2;
		}

		ROS_DEBUG_STREAM("t_curr: " << t_curr);
	}

	seg = solveSegment(constraints, t_curr); // solve to segment at curr
	if(testSegmentForFeasibilityFAST(seg, physical)) // if the segment is too slow
	{
		return seg;
	}
	else
	{
		return high_seg;
	}

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
 * evaluates from t0 to tf
 */
bool TrajectoryGenerator::testSegmentForFeasibilityFAST(TrajectorySegment seg, PhysicalCharacterisics physical)
{
	EfficientTrajectorySegment eff_seg = preComputeTrajectorySegment(seg);

	/*
	//first check if quad is in free fall
	if(!checkForces(calculateMotorForces(eff_seg, physical, polyMinTime(eff_seg.accel.z, 0, eff_seg.accel.tf)), physical)){
		ROS_DEBUG_STREAM("FREE FALL");
		return false; // the quad would need to be inverted to fly this!
	}

	double t = 0;
	// get the point of maximum acceleration
	polyVectorMaxFAST(eff_seg.accel.x, eff_seg.accel.y, eff_seg.accel.z, 0, eff_seg.accel.tf, t);

	//check the forces at that point
	if(!checkForces(calculateMotorForces(eff_seg, physical, t), physical)){
		ROS_DEBUG_STREAM("FORCE TOO HIGH");
		return false; // the quad would need too much force
	}

	// get the point of maximum snap
	polyVectorMaxFAST(eff_seg.snap.x, eff_seg.snap.y, eff_seg.snap.z, 0, eff_seg.snap.tf, t);

	//check the forces at that point
	if(!checkForces(calculateMotorForces(eff_seg, physical, t), physical)){
		ROS_DEBUG_STREAM("TORQUE TOO HIGH");
		return false; // the quad would need too much torque
	}*/

	for(double t = seg.t0; t < seg.tf + FEASIBILITY_DT_FAST; t += FEASIBILITY_DT_FAST)
	{
		if(!checkForces(calculateMotorForces(eff_seg, physical, t), physical))
		{
			ROS_DEBUG_STREAM("failed force test: " << calculateMotorForces(eff_seg, physical, t));
			return false;
		}
	}

	return true;
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

	//calculate the moments required
	Eigen::Vector3d moment_body = physical.J * omega_dot_body + omega_body.cross(physical.J * omega_body);

	Eigen::Vector4d b;
	b << f_total, moment_body(0), moment_body(1), moment_body(2);

	return physical.torqueTransition_inv * b;

}

/*
 * checks if forces are within bounds
 */
bool TrajectoryGenerator::checkForces(Eigen::Vector4d forces, PhysicalCharacterisics& physical)
{
	if(forces(0) > physical.max_motor_thrust || forces(0) < physical.min_motor_thrust){
		return false;
	}
	if(forces(1) > physical.max_motor_thrust || forces(1) < physical.min_motor_thrust){
		return false;
	}
	if(forces(2) > physical.max_motor_thrust || forces(2) < physical.min_motor_thrust){
		return false;
	}
	if(forces(3) > physical.max_motor_thrust || forces(3) < physical.min_motor_thrust){
		return false;
	}

	return true;
}

nav_msgs::Path generateTrajectorySegmentPath(TrajectorySegment seg)
{

}
