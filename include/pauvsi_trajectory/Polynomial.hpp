/*
 * Polynomial.h
 *
 *  Created on: Apr 14, 2017
 *      Author: pauvsi
 */

#ifndef PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_POLYNOMIAL_HPP_
#define PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_POLYNOMIAL_HPP_

#include <eigen3/Eigen/Geometry>
#include <vector>
#include "Types.h"
#include <iostream>
#include "ros/ros.h"

#include "Config.h"

/*
 * finds the derivative of polynomial coefficients
 */
Polynomial polyDer(Polynomial in)
{
	int size = in.size();
	Polynomial new_poly(size - 1, 1);
	for(int i=0; i < size-1; i++){
		new_poly(i) = ((size - 1 - i)*in(i));
	}
	return new_poly;
}

double polyVal(Polynomial in, double t)
{
	double result=0;
	double t_agg = 1;
	for (int i = in.size() - 1; i >= 0; i--){
		result += t_agg * in(i);
		t_agg *= t;
	}
	return result;
}

/*
 * returns the time at which an absolute maximum is found
 */
double polyMaxTime(Polynomial& in, double t0, double tf){
	double max = 0;
	double maxT = t0;
	double temp;
	for(double t = t0; t < tf; t += POLYMAX_DT)
	{
		temp = std::fabs(polyVal(in, t));
		if(temp > max)
		{
			max = temp;
			maxT = t;
		}
	}

	return maxT;
}

/*
 * returns the vector at maximum absolute
 * reference return of time
 */
Eigen::Vector3d polyVectorMax(Polynomial& x, Polynomial& y, Polynomial& z, double t0, double tf, double& t_out)
{
	ROS_ASSERT(x.size() == y.size() && y.size() == z.size());
	double max = 0;
	double maxT = t0;
	double tempMax = 0;
	Eigen::Vector3d maxVec;
	Eigen::Vector3d tempVec;

	for(double t = t0; t < tf; t += POLYMAX_DT)
	{
		tempVec(0) = polyVal(x, t);
		tempVec(1) = polyVal(y, t);
		tempVec(2) = polyVal(z, t);
		tempMax = tempVec.squaredNorm();
		if(tempMax > max)
		{
			max = tempMax;
			maxVec = tempVec;
			maxT = t;
		}
	}

	t_out = maxT;
	return maxVec;
}

void polyRoot(Polynomial in, double t0, double tf){

}



#endif /* PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_POLYNOMIAL_HPP_ */
