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

TrajectorySegment TrajectoryGenerator::computeHighOrderMinimumTimeTrajectory(DynamicTrajectoryConstraints constraints, PhysicalCharacterisics phys)
{
	// compute initial guess for times
	constraints.start.t = 0.0;
	if(constraints.middle.size())
	{
		constraints.middle.front().t = ((constraints.middle.front().pos.toEigen() - constraints.start.pos.toEigen()).norm() * DIST2DT_MULTIPLIER);

		for(int i = 1; i < constraints.middle.size(); i++)
		{
			constraints.middle[i].t = ((constraints.middle[i].pos.toEigen() - constraints.middle[i-1].pos.toEigen()).norm() * DIST2DT_MULTIPLIER) + constraints.middle[i-1].t;
		}

		constraints.end.t = ((constraints.end.pos.toEigen() - constraints.middle.back().pos.toEigen()).norm() * DIST2DT_MULTIPLIER) + constraints.middle.back().t;
	}
	else
	{
		constraints.end.t = (constraints.end.pos.toEigen() - constraints.start.pos.toEigen()).norm() * DIST2DT_MULTIPLIER;
	}

	ROS_DEBUG_STREAM("intial time guess: " << constraints.getTimes());

	TrajectorySegment seg = solveSegment(constraints);

	ROS_DEBUG_STREAM("size 1: " << seg.x.size());

	seg = this->computeGeometricallyFeasibleTrajectory(constraints);

	ROS_DEBUG_STREAM("size 2: " << seg.x.size());

	seg = this->minimizeTimeFAST(constraints, phys);

	ROS_DEBUG_STREAM("size 3: " << seg.x.size());

	if(DT_OPTIM_ACCURATE)
	{
		seg = this->minimizeTimeACCURATE(constraints, phys);
	}

	ROS_DEBUG_STREAM("size 4: " << seg.x.size());

	return seg;
}

TrajectorySegment TrajectoryGenerator::minimizeTimeFAST(DynamicTrajectoryConstraints& constraints, PhysicalCharacterisics phys)
{
	double d2dt_curr = DIST2DT_MULTIPLIER;
	Eigen::VectorXd current_times = constraints.getTimes(); // stores the current best
	Eigen::VectorXd test_times = current_times;

	TrajectorySegment seg, last_working_seg;
	seg = solveSegment(constraints);
	last_working_seg = seg;

	// if not fast optimize each segment instead of whole path
	int level = 1;
	/*if(!D2DT_OPTIM_FAST)
	{
		level = current_times.size() - 1;
	}*/

	ROS_DEBUG_STREAM("times size: " << current_times.size());

	for(int i = 0; i < level; i++)
	{
		double d2dt_high = DIST2Dt_MAX;
		double d2dt_low = DIST2DT_MIN;
		bool pass = true;

		for(int j = 0; j < D2DT_TIME_OPTIM_ITER; j++)
		{
			ROS_DEBUG_STREAM("size before test: " << seg.x.size());
			pass = this->testSegmentForFeasibilityFAST(seg, phys);
			ROS_DEBUG_STREAM("size after test: " << seg.x.size());

			ROS_ERROR_COND(j == 0 && !pass, "TRAJECTORY MAY NOT BE FEASIBLE");
			if(pass)
			{
				d2dt_high = d2dt_curr;
				d2dt_curr = d2dt_low + 0.5*(d2dt_curr - d2dt_low);
				last_working_seg = seg;
			}
			else
			{
				d2dt_low = d2dt_curr;
				d2dt_curr = d2dt_high - 0.5*(d2dt_high - d2dt_curr);
			}
			ROS_DEBUG_STREAM("iter " << j+1 << " d2dt " << d2dt_curr);

			ROS_DEBUG_STREAM("times size: " << current_times.size());
			ROS_DEBUG_STREAM("test times size: " << test_times.size());
			if(j == D2DT_TIME_OPTIM_ITER - 1){
				d2dt_curr = d2dt_high;
				test_times.segment(i, test_times.size()-i) = d2dt_curr * current_times.segment(i, current_times.size()-i);
				current_times = test_times;

				ROS_DEBUG_STREAM("using: " << d2dt_curr);
				seg = last_working_seg;
				constraints.assignTimes(current_times);
				break;
			}
			else{
				test_times.segment(i, test_times.size()-i) = d2dt_curr * current_times.segment(i, current_times.size()-i);
				ROS_DEBUG_STREAM("testing times: " << test_times.transpose());
				constraints.assignTimes(test_times);
				seg = solveSegment(constraints);
			}
		}
	}

	ROS_DEBUG("finished optim");

	return seg;
}

/*
 * expects that the starting constraints work
 */
TrajectorySegment TrajectoryGenerator::minimizeTimeACCURATE(DynamicTrajectoryConstraints& constraints, PhysicalCharacterisics phys)
{
	double dt_curr = 0;
	Eigen::VectorXd current_times = constraints.getTimes(); // stores the current best
	Eigen::VectorXd test_times = current_times;

	TrajectorySegment seg, last_working_seg;
	seg = solveSegment(constraints);



	//ROS_DEBUG_STREAM("times size: " << current_times.size());

	for(int i = 1; i < current_times.size(); i++)
	{
		dt_curr = 0;
		double dt_high = ACCURATE_DT_HIGH;
		double dt_low = ACCURATE_DT_LOW;
		if(current_times(i-1)-current_times(i) < dt_low)
		{
			ROS_DEBUG_STREAM("CORRECTING");
			dt_low = current_times(i-1)-current_times(i);
		}

		bool pass = true;

		for(int j = 0; j < ACCURATE_TIME_OPTIM_ITERS; j++)
		{
			pass = this->testSegmentForFeasibilityFAST(seg, phys);

			if(j == 0 && !pass)
			{
				ROS_ERROR_STREAM("ACCURATE DT OPTIM FAILED AT SEGMENT " << i << " DT STARTED IN A FAILING STATE");
			}

			if(pass)
			{
				dt_high = dt_curr;
				dt_curr = dt_low + 0.5*(dt_curr - dt_low);
				last_working_seg = seg;
			}
			else
			{
				dt_low = dt_curr;
				dt_curr = dt_high - 0.5*(dt_high - dt_curr);
			}
			ROS_DEBUG_STREAM("iter " << j+1 << " dt " << dt_curr);

			//ROS_DEBUG_STREAM("times size: " << current_times.size());
			//ROS_DEBUG_STREAM("test times size: " << test_times.size());
			if(j == ACCURATE_TIME_OPTIM_ITERS - 1){
				dt_curr = dt_high;
				test_times.segment(i, test_times.size()-i) = dt_curr * Eigen::VectorXd::Ones(current_times.size()-i) + current_times.segment(i, current_times.size()-i);
				current_times = test_times;

				ROS_DEBUG_STREAM("using: " << dt_curr);
				seg = last_working_seg;
				constraints.assignTimes(current_times);
				break;
			}
			else{
				test_times.segment(i, test_times.size()-i) = dt_curr * Eigen::VectorXd::Ones(current_times.size()-i) + current_times.segment(i, current_times.size()-i);
				ROS_DEBUG_STREAM("testing times: " << test_times.transpose());
				constraints.assignTimes(test_times);
				seg = solveSegment(constraints);
			}
		}
	}

	return seg;
}

/*
 * finds a geometrically feasible trajectory
 * and updates the constraints by reference
 */
TrajectorySegment TrajectoryGenerator::computeGeometricallyFeasibleTrajectory(DynamicTrajectoryConstraints& constraints)
{

	bool pass = true;

	TrajectorySegment seg = this->solveSegment(constraints);
	pass = this->refineGeometricConstraints(seg, constraints);
	if(pass)
		return seg;


	for(int i = 1; i < MAX_GEOM_REFINE_ITER; i++)
	{
		seg = this->solveSegment(constraints);
		pass = this->refineGeometricConstraints(seg, constraints);
		if(pass)
			break;
	}

	if(!pass)
		ROS_ERROR("TRAJECTORY MAY BE DANGEROUS!!!!!!!");

	return seg;
}

std::vector<BasicWaypointConstraint> TrajectoryGenerator::simplifyConstraints(DynamicTrajectoryConstraints constraints){
	std::vector<BasicWaypointConstraint> simple;

	simple.push_back(BasicWaypointConstraint(constraints.start.pos, constraints.start.t));
	simple.back().geoConstraint = constraints.start.geoConstraint;
	for(auto e : constraints.middle)
	{
		simple.push_back(e);
	}
	simple.push_back(BasicWaypointConstraint(constraints.end.pos, constraints.end.t));
	simple.back().geoConstraint = constraints.end.geoConstraint;
	return simple;
}

/*
 * adds waypoints as necesary and returns whether it passed
 */
bool TrajectoryGenerator::refineGeometricConstraints(TrajectorySegment seg, DynamicTrajectoryConstraints& constraints)
{
	double t_fail = 0;
	bool pass = true;
	if(constraints.middle.size())
	{
		std::vector<BasicWaypointConstraint> new_middle;


		//set the test time
		seg.t0 = constraints.start.t;
		seg.tf = constraints.middle.at(0).t;
		ROS_DEBUG_STREAM("bounds: " << seg.t0 << " => " << seg.tf);
		if(!this->testSegmentForGeometricFeasibility(seg, constraints.start.geoConstraint, t_fail))
		{
			Point pt = Point(constraints.start.pos.toEigen() + 0.5 * (constraints.middle.at(0).pos.toEigen() - constraints.start.pos.toEigen()));
			double t = constraints.start.t + 0.5 * (constraints.middle.at(0).t - constraints.start.t);
			new_middle.push_back(BasicWaypointConstraint(pt, t));
			new_middle.back().geoConstraint = constraints.start.geoConstraint;

			ROS_DEBUG_STREAM("added pt: " << pt.toEigen().transpose() << " at t = " << t);

			pass = false;
		}
		new_middle.push_back(constraints.middle.at(0));

		for(int i = 0; i < (constraints.middle.size() - 1); i++)
		{
			//set the test time
			seg.t0 = constraints.middle.at(i).t;
			seg.tf = constraints.middle.at(i+1).t;
			ROS_DEBUG_STREAM("bounds: " << seg.t0 << " => " << seg.tf);
			if(!this->testSegmentForGeometricFeasibility(seg, constraints.middle.at(i).geoConstraint, t_fail))
			{
				Point pt = Point(constraints.middle.at(i).pos.toEigen() + 0.5 * (constraints.middle.at(i+1).pos.toEigen() - constraints.middle.at(i).pos.toEigen()));
				double t = constraints.middle.at(i).t + 0.5 * (constraints.middle.at(i+1).t - constraints.middle.at(i).t);
				new_middle.push_back(BasicWaypointConstraint(pt, t));
				new_middle.back().geoConstraint = constraints.middle.at(i).geoConstraint;

				ROS_DEBUG_STREAM("added pt: " << pt.toEigen().transpose() << " at t = " << t);

				pass = false;
			}
			new_middle.push_back(constraints.middle.at(i+1));
		}

		//set the test time
		seg.t0 = constraints.middle.back().t;
		seg.tf = constraints.end.t;
		ROS_DEBUG_STREAM("bounds: " << seg.t0 << " => " << seg.tf);
		if(!this->testSegmentForGeometricFeasibility(seg, constraints.middle.back().geoConstraint, t_fail))
		{
			Point pt = Point(constraints.middle.back().pos.toEigen() + 0.5 * (constraints.end.pos.toEigen() - constraints.middle.back().pos.toEigen()));
			double t = constraints.middle.back().t + 0.5 * (constraints.end.t - constraints.middle.back().t);
			constraints.middle.push_back(BasicWaypointConstraint(pt, t));
			new_middle.back().geoConstraint = constraints.middle.back().geoConstraint;

			ROS_DEBUG_STREAM("added pt: " << pt.toEigen().transpose() << " at t = " << t);

			pass = false;
		}

		constraints.middle = new_middle;
	}
	else
	{
		//set the test time
		seg.t0 = constraints.start.t;
		seg.tf = constraints.end.t;
		ROS_DEBUG_STREAM("bounds: " << seg.t0 << " => " << seg.tf);
		if(!this->testSegmentForGeometricFeasibility(seg, constraints.start.geoConstraint, t_fail))
		{
			Point pt = Point(constraints.start.pos.toEigen() + 0.5 * (constraints.end.pos.toEigen() - constraints.start.pos.toEigen()));
			double t = constraints.start.t + 0.5 * (constraints.end.t - constraints.start.t);
			constraints.middle.push_back(BasicWaypointConstraint(pt, t));
			constraints.middle.back().geoConstraint = constraints.start.geoConstraint;

			ROS_DEBUG_STREAM("added pt: " << pt.toEigen().transpose() << " at t = " << t);

			pass = false;
		}
	}

	return pass;
}

bool TrajectoryGenerator::testSegmentForGeometricFeasibility(TrajectorySegment seg, std::vector<GeometricConstraint> geoConstraints, double& failureTime)
{
	if(geoConstraints.size() == 0) // if this trajectory has no constraints it automatically passes
	{
		return true;
	}

	for(double t = seg.t0; t < seg.tf; t += FEASIBILITY_DT_FAST)
	{
		double z = polyVal(seg.z, t);

		//ROS_DEBUG_STREAM("t = " << t);
		//ROS_DEBUG_STREAM("z = " << z);

		for(auto e : geoConstraints)
		{
			if(e.type == e.PLANE_MIN)
			{
				if(z < e.z_min)
				{
					failureTime = t;
					ROS_DEBUG_STREAM("-failed at " << t);
					return false;
				}
			}
			else if(e.type == e.PLANE_MAX)
			{
				if(z > e.z_max)
				{
					failureTime = t;
					ROS_DEBUG_STREAM("+failed at " << t);
					return false;
				}
			}
		}
	}

	return true;
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

	//ROS_DEBUG_STREAM("dynamic matrix size is " << dim);

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
			//ROS_DEBUG_STREAM("addressing: " << 5 + i << ", " << dim - 1 - j);
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
			//ROS_DEBUG_STREAM("addressing: " << 5 + i + midPointCount << ", " << j);
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

TrajectorySegment TrajectoryGenerator::solveSegment(DynamicTrajectoryConstraints constraints)
{
	TrajectorySegment seg;
	seg.t0 = constraints.start.t;
	seg.tf = constraints.end.t;

	Eigen::MatrixXd A_inv = generateDynamicPolyMatrix(constraints).lu().inverse();

	Eigen::VectorXd b(10 + constraints.middle.size(), 1);

	int midPointCount = constraints.middle.size();

	b(0) = constraints.start.pos.x;
	b(1) = constraints.start.vel.x;
	b(2) = constraints.start.accel.x;
	b(3) = constraints.start.jerk.x;
	b(4) = constraints.start.snap.x;

	for(int i = 0; i < midPointCount; i++)
	{
		b(5 + i) = constraints.middle.at(i).pos.x;
		//ROS_DEBUG("ran midpoint");
	}

	b(5 + midPointCount) = constraints.end.pos.x;
	b(6 + midPointCount) = constraints.end.vel.x;
	b(7 + midPointCount) = constraints.end.accel.x;
	b(8 + midPointCount) = constraints.end.jerk.x;
	b(9 + midPointCount) = constraints.end.snap.x;

	//ROS_DEBUG_STREAM("b for x: " << b.transpose());

	seg.x = A_inv * b; // solve for x poly

	//-=-==-=-=-=-=-=-=-=-=-=-=-=-=
	b(0) = constraints.start.pos.y;
	b(1) = constraints.start.vel.y;
	b(2) = constraints.start.accel.y;
	b(3) = constraints.start.jerk.y;
	b(4) = constraints.start.snap.y;

	for(int i = 0; i < midPointCount; i++)
	{
		b(5 + i) = constraints.middle.at(i).pos.y;
	}

	b(5 + midPointCount) = constraints.end.pos.y;
	b(6 + midPointCount) = constraints.end.vel.y;
	b(7 + midPointCount) = constraints.end.accel.y;
	b(8 + midPointCount) = constraints.end.jerk.y;
	b(9 + midPointCount) = constraints.end.snap.y;

	//ROS_DEBUG_STREAM("b for y: " << b.transpose());

	seg.y = A_inv * b; // solve for y poly


	//-=-==-=-=-=-=-=-=-=-=-=-=-=-=
	b(0) = constraints.start.pos.z;
	b(1) = constraints.start.vel.z;
	b(2) = constraints.start.accel.z;
	b(3) = constraints.start.jerk.z;
	b(4) = constraints.start.snap.z;

	for(int i = 0; i < midPointCount; i++)
	{
		b(5 + i) = constraints.middle.at(i).pos.z;
	}

	b(5 + midPointCount) = constraints.end.pos.z;
	b(6 + midPointCount) = constraints.end.vel.z;
	b(7 + midPointCount) = constraints.end.accel.z;
	b(8 + midPointCount) = constraints.end.jerk.z;
	b(9 + midPointCount) = constraints.end.snap.z;

	//ROS_DEBUG_STREAM("b for z: " << b.transpose());

	seg.z = A_inv * b; // solve for y poly

	return seg;
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

	Eigen::Vector3d F_dot_inertial_bar = (F_dot_inertial / f_total) - ((F_inertial * F_inertial.transpose() * F_dot_inertial) / (f_total*f_total*f_total)); // equation 3.20 in Cutler's paper


	//calculate the desired angular rate
	Eigen::Vector3d tempCross = F_inertial_bar.cross(F_dot_inertial_bar);

	Eigen::Vector3d omega_body;
	omega_body << tempCross(0), tempCross(1), 0; // project onto the x-y plane

	//calculate the second derivative of inertial frame force unit vector
	Eigen::Vector3d F_dot_dot_inertial = physical.mass * snap;

	Eigen::Vector3d F_dot_dot_inertial_bar = (F_dot_dot_inertial / f_total) -
			((2 * F_dot_inertial * F_inertial.transpose() * F_dot_inertial   +   F_inertial * F_dot_inertial.transpose() * F_dot_inertial    +
					F_inertial * F_inertial.transpose() * F_dot_dot_inertial) / (f_total*f_total*f_total))  +
					((3 * F_inertial * F_inertial.transpose() * F_dot_inertial) / (f_total*f_total*f_total*f_total*f_total));

	// calculate the angular acceleration now
	tempCross = F_inertial_bar.cross((F_dot_dot_inertial_bar - omega_body.cross(omega_body.cross(F_dot_inertial_bar))));

	Eigen::Vector3d omega_dot_body;
	omega_dot_body << tempCross(0), tempCross(1), 0; // this is the body angular acceleration about the x and y axis

	//calculate the moments required
	Eigen::Vector3d moment_body = physical.J * omega_dot_body + omega_body.cross(physical.J * omega_body);

	Eigen::Vector4d b;
	b << f_total, moment_body(0), moment_body(1), moment_body(2);

	//ROS_DEBUG_STREAM("accel should be: " << accel.transpose());

	return physical.torqueTransition_inv * b;

}

/*
 * for visulization only
 */
Eigen::Quaterniond TrajectoryGenerator::calculateRotation(TrajectorySegment accel_poly, double t)
{
	Eigen::Vector3d accel;
	accel << polyVal(accel_poly.x, t), polyVal(accel_poly.y, t), polyVal(accel_poly.z, t);

	Eigen::Vector3d F_inertial = accel;
	F_inertial(2) += G; // add the FORCE due to gravity

	double f_total = F_inertial.norm(); // the total force required at max acceleration

	ROS_DEBUG_STREAM("rot f_tot: " << f_total);
	Eigen::Vector3d F_inertial_bar = F_inertial / f_total; // should find the direction vector of f inertial

	Eigen::Vector3d F_body_bar;
	F_body_bar << 0, 0, 1;

	double quatNorm = 1 / sqrt(2.0*(1 + F_inertial_bar.transpose() * F_body_bar));

	Eigen::Quaterniond quat;

	quat.w() = 1 + F_inertial_bar.transpose() * F_body_bar;

	Eigen::Vector3d temp;
	temp = F_inertial_bar.cross(F_body_bar);

	quat.x() = -temp(0);
	quat.y() = -temp(1);
	quat.z() = -temp(2);

	return quat;
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

nav_msgs::Path TrajectoryGenerator::generateTrajectorySegmentPath(TrajectorySegment seg)
{
	nav_msgs::Path path;

	TrajectorySegment accel;

	ROS_DEBUG("running polyder");
	accel.x = polyDer(polyDer(seg.x));
	accel.y = polyDer(polyDer(seg.y));
	accel.z = polyDer(polyDer(seg.z));
	ROS_DEBUG("ran polyder");

	for(double t = seg.t0; t < seg.tf; t += VISUALIZE_DT)
	{
		geometry_msgs::PoseStamped pose;

		pose.pose.position.x = polyVal(seg.x, t);
		pose.pose.position.y = polyVal(seg.y, t);
		pose.pose.position.z = polyVal(seg.z, t);

		Eigen::Quaterniond quat = this->calculateRotation(accel, t);

		pose.pose.orientation.w = quat.w();
		pose.pose.orientation.x = quat.x();
		pose.pose.orientation.y = quat.y();
		pose.pose.orientation.z = quat.z();

		pose.header.stamp = ros::Time(t);

		path.poses.push_back(pose);
	}

	path.header.frame_id = "world";

	return path;
}

/*
 * this is the brute force approximate way
 */
double TrajectoryGenerator::arcLengthTrajectoryBRUTE(TrajectorySegment seg)
{
	TrajectorySegment vel = polyDer(seg);

	double dist = 0;

	for(double t = seg.t0; t < seg.tf; t += ARCLENGTH_DT)
	{
		//ROS_DEBUG_STREAM(t);
		dist += polyVal(vel, t).norm() * ARCLENGTH_DT;
	}

	return dist;
}
