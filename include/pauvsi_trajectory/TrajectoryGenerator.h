/*
 * TrajectoryGenerator.h
 *
 *  Created on: Nov 5, 2016
 *      Author: kevin
 */

#ifndef PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TRAJECTORYGENERATOR_H_
#define PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TRAJECTORYGENERATOR_H_

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>

#include <eigen3/Eigen/Geometry>

#include <nav_msgs/Path.h>

#include "Types.h"
#include "Polynomial.hpp"


class TrajectoryGenerator {
public:
	TrajectoryGenerator();
	virtual ~TrajectoryGenerator();

	Eigen::MatrixXd generateDynamicPolyMatrix(DynamicTrajectoryConstraints constraints);
	TrajectorySegment solveSegment(DynamicTrajectoryConstraints constraints);

	Eigen::Matrix<double, 10, 10> generatePolyMatrix(double tf);
	Polynomial solvePoly(PolynomialConstraints constraints, Eigen::Matrix<double, 10, 10> A_inv);
	TrajectorySegment solveSegment(TrajectoryConstraints constraints, double tf);

	TrajectorySegment computeMinimumTimeTrajectorySegment(TrajectoryConstraints constraints, PhysicalCharacterisics physical, double tf_guess);

	bool testSegmentForFeasibilityFAST(TrajectorySegment seg, PhysicalCharacterisics physical);

	Eigen::Vector4d calculateMotorForces(EfficientTrajectorySegment ts, PhysicalCharacterisics physical, double t);

	EfficientTrajectorySegment preComputeTrajectorySegment(TrajectorySegment pos);

	bool checkForces(Eigen::Vector4d forces, PhysicalCharacterisics& physical);

	nav_msgs::Path generateTrajectorySegmentPath(TrajectorySegment seg);


};

#endif /* PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TRAJECTORYGENERATOR_H_ */
