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


#endif /* PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TYPES_H_ */
