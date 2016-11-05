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

class TrajectoryGenerator {
public:
	TrajectoryGenerator();
	virtual ~TrajectoryGenerator();
};

#endif /* PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_TRAJECTORYGENERATOR_H_ */
