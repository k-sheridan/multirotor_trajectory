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

/*
 * finds the derivative of polynomial coefficients
 */
Polynomial polyDer(Polynomial in)
{
	int size = in.size();
	std::vector<double> new_poly;
	for(int i=0; i < size-1; i++){
		new_poly.push_back((size-1)*in[i]);
	}
	return new_poly;
}

double polyVal(Polynomial in, double t)
{
	double result=0;
	double t_agg = 1;
	for (Polynomial::iterator it = in.end() - 1; it != in.begin(); it--){
		result += t_agg * (*it);
		t_agg *= t;
	}
	return result;
}

void polyRoot(Polynomial in, double t0, double tf){

}



#endif /* PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_POLYNOMIAL_HPP_ */
