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

#include <unsupported/Eigen/Polynomials>
#include <iostream>

/*
 * finds the derivative of polynomial coefficients
 */
std::vector<double> polyDer(std::vector<double> in)
{
	int size = in.size();
	std::vector<double> new_poly;
	for(int i=0; i < size-1; i++){
		new_poly.push_back((size-1)*in[i]);
	}
	return new_poly;
}

double polyVal(std::vector<int> in, double t)
{
	double result=0;
	int size = in.size();
	for (int i=0; i < size; i++){
		result += in[i]*pow(t,size-i);
	}
	return result;
}

void polyRoot(std::vector<double> in){
	Eigen::Vector4d roots = Eigen::Vector4d::Random();
	cout << "Roots: " << roots.transpose() << endl;
	Eigen::Matrix<double,5,1> polynomial;
	roots_to_monicPolynomial( roots, polynomial );
	cout << "Polynomial: ";
	for( int i=0; i<4; ++i ){ cout << polynomial[i] << ".x^" << i << "+ "; }
	cout << polynomial[4] << ".x^4" << endl;
	Eigen:: Vector4d evaluation;
	for( int i=0; i<4; ++i ){
		evaluation[i] = poly_eval( polynomial, roots[i] ); }
	cout << "Evaluation of the polynomial at the roots: " << evaluation.transpose();
	return evaluation;
}



#endif /* PAUVSI_M7_PAUVSI_TRAJECTORY_INCLUDE_PAUVSI_TRAJECTORY_POLYNOMIAL_HPP_ */
