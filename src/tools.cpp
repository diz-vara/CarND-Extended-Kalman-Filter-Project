#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() == 0) {
		std::cerr << "Zero vector length" << std::endl;
		return rmse;
	}

	int len = estimations.size();
	if (len != ground_truth.size()) {
		std::cerr << "Vector lengths are not equal" << std::endl;
		return rmse;

	}

	//accumulate squared residuals
	for (int i = 0; i < len; ++i) {
		VectorXd diff = estimations[i] - ground_truth[i];
		diff = diff.array() * diff.array();
		rmse = rmse + diff;
	}

	//calculate the mean
	rmse = rmse / len;

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  //std::cout << "CalculateJacobian from x: " << x_state << std::endl;

	//check division by zero
	if (px == 0 && py == 0) {
		std::cerr << "Error: divide by zero!";
	}
	else {
		double sumPxy2 = px*px + py * py;
		double sqrt_sumPxy2 = sqrt(sumPxy2);
		double sumPxy2_3_2 = sqrt(sumPxy2 * sumPxy2 * sumPxy2);

		Hj(0, 0) = px / sqrt_sumPxy2;
		Hj(0, 1) = py / sqrt_sumPxy2;
		Hj(0, 2) = 0;
		Hj(0, 3) = 0; 

		Hj(1, 0) = -1 * py / sumPxy2;
		Hj(1, 1) = px / sumPxy2;
		Hj(1, 2) = 0;
		Hj(1, 3) = 0;

		Hj(2, 0) = py * (vx * py - vy * px) / sumPxy2_3_2;
		Hj(2, 1) = px * (vy * px - vx * py) / sumPxy2_3_2;
		Hj(2, 2) = px / sqrt_sumPxy2;;
		Hj(2, 3) = py / sqrt_sumPxy2;;

	}

  //std::cout << "Jacobian: " << Hj << std::endl;

	return Hj;
}
