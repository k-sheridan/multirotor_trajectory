/*
 * Physics.h
 *
 *  Created on: May 22, 2017
 *      Author: kevin
 */

#ifndef PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_PHYSICS_H_
#define PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_PHYSICS_H_


//this is the 4x4 matrix which relates torque to motor forces
// use meters
#define TORQUE_TRANSITION 1, 1, 1, 1, -0.25, -0.25, 0.25, 0.25, -0.25, 0.25, 0.25, -0.25, -0.01, 0.01, -0.01, 0.01

// these are the max and min individual motor forces.
// you can tune the aggression of the quad using the max in Newtons
#define MOTOR_FORCE_MIN 1
#define MOTOR_FORCE_MAX 20

//mass of quad kg
#define MASS 5

//moment of inertia matrix kg*m^2
#define J_MATRIX 0.75, 0, 0, 0, 0.75, 0, 0, 0, 0.75

#define G 9.81

#define RANDOM_SEED 1


#endif /* PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_PHYSICS_H_ */
