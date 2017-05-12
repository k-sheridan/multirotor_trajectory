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
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
			tf_9,         tf_8,         tf_7,         tf_6,         tf_5,       tf_4,     tf_3,   tf_2, tf, 1,
			9*tf_8,       8*tf_7,       7*tf_6,       6*tf_5,       5*tf_4,     4*tf_3,   3*tf_2, 2*tf, 1,  0,
			72*tf_7,     56*tf_6,     42*tf_5,     30*tf_4,     20*tf_3,   12*tf_2, 6*tf, 1,    0,  0,
			504*tf_6,   336*tf_5,   210*tf_4,   120*tf_3,   60*tf_2, 24*tf, 1,      0,    0,  0,
			3024*tf_5, 1680*tf_4, 840*tf_3, 360*tf_2, 120*tf, 1, 0, 0, 0, 0;
	return A;
}

