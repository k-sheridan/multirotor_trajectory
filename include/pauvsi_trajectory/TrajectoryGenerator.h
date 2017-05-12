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

#include "Types.h"


class TrajectoryGenerator {
public:
	TrajectoryGenerator();
	virtual ~TrajectoryGenerator();

	Eigen::Matrix<double, 10, 10> generatePolyMatrix(double tf);
};

#endif /* PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TRAJECTORYGENERATOR_H_ */
