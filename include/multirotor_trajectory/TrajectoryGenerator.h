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
#include "Physics.h"


class TrajectoryGenerator {
public:
	TrajectoryGenerator();
	virtual ~TrajectoryGenerator();

	Eigen::MatrixXd generateDynamicPolyMatrix(DynamicTrajectoryConstraints constraints);
	TrajectorySegment solveSegment(DynamicTrajectoryConstraints constraints);
	TrajectorySegment computeHighOrderMinimumTimeTrajectory(DynamicTrajectoryConstraints constraints, PhysicalCharacterisics phys);

	TrajectorySegment minimizeTimeFAST(DynamicTrajectoryConstraints& constraints, PhysicalCharacterisics phys);
	TrajectorySegment minimizeTimeACCURATE(DynamicTrajectoryConstraints& constraints, PhysicalCharacterisics phys);

	std::vector<BasicWaypointConstraint> simplifyConstraints(DynamicTrajectoryConstraints constraints);

	bool refineGeometricConstraints(TrajectorySegment seg, DynamicTrajectoryConstraints& constraints);

	TrajectorySegment computeGeometricallyFeasibleTrajectory(DynamicTrajectoryConstraints& constraints);


	bool testSegmentForGeometricFeasibility(TrajectorySegment seg, std::vector<GeometricConstraint> geoConstraints, double& failureTime);

	Eigen::Matrix<double, 10, 10> generatePolyMatrix(double tf);
	Polynomial solvePoly(PolynomialConstraints constraints, Eigen::Matrix<double, 10, 10> A_inv);
	TrajectorySegment solveSegment(TrajectoryConstraints constraints, double tf);

	TrajectorySegment computeMinimumTimeTrajectorySegment(TrajectoryConstraints constraints, PhysicalCharacterisics physical, double tf_guess);

	bool testSegmentForFeasibilityFAST(TrajectorySegment seg, PhysicalCharacterisics physical);

	Eigen::Vector4d calculateMotorForces(EfficientTrajectorySegment ts, PhysicalCharacterisics physical, double t);

	Eigen::Quaterniond calculateRotation(TrajectorySegment accel, double t);

	double arcLengthTrajectoryBRUTE(TrajectorySegment seg);

	EfficientTrajectorySegment preComputeTrajectorySegment(TrajectorySegment pos);

	bool checkForces(Eigen::Vector4d forces, PhysicalCharacterisics& physical);

	nav_msgs::Path generateTrajectorySegmentPath(TrajectorySegment seg);


};

#endif /* PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TRAJECTORYGENERATOR_H_ */
