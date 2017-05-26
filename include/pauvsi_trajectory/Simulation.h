/*
 * Simulation.h
 *
 *  Created on: May 22, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_SIMULATION_H_
#define PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_SIMULATION_H_

#define MOTOR_FORCE SIGMA 0.2
#define POSITION_SIGMA 0.02
#define OMEGA_SIGMA 0.0005
#define VEL_SIGMA 0.01
#define QUAT_SIGMA 0.001

#define PHYSICS_UPDATE_DT 0.01

#define MOTOR_FORCE_TOPIC "forceRequest"
#define POSE_TOPIC "state/pose"
#define TWIST_TOPIC "state/twist"

#define BASE_FRAME "base_link"
#define WORLD_FRAME "world"

#define START_POS -9, -9, 0.5

#define MOTOR_ABS_MAX 25
#define MOTOR_ABS_MIN 0.1



#endif /* PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_SIMULATION_H_ */
