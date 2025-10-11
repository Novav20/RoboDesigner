#include <iostream>
#include <RoboDesignerCore/MathUtils.h>

int main() {
	Eigen::Matrix2d matrix_a;
	Eigen::Matrix2d matrix_b;

	matrix_a << 1, 2,
		3, 4;

	matrix_b << 5, 6,
		7, 8;

	std::cout << "a * b = \n" << MathUtils::multiplyMatrices(matrix_a, matrix_b) << std::endl << std::endl;

	return 0;
}