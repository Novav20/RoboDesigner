#include <iostream>
#include <Eigen/Dense>

int main(){
	Eigen::Matrix3d m;

	m << 1, 2, 3,
		4, 5, 6,
		7, 8, 9;

	std::cout << "Successfully created and initialized an Eigen Matrix:" << std::endl;
	std::cout << m << std::endl;

	return 0;
}