/*
 * soc_kf_eigen.cpp
 *
 *  Created on: Sep 22, 2026
 *      Author: Madelyn
 */


#include "soc_kf_eigen.h"
#include "Eigen/Core"

void will_eigen_compile() {
	Eigen::Matrix<float, 3, 3> I {
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1}
	};

	Eigen::Vector<float, 3> x {
			1,
			2,
			3
	};

	auto result = I * x;
	(void)result;
}
