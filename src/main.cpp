#include <iostream>
#include <Eigen/Dense>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

int main() {
	Eigen::Matrix3d m;

	m << 1, 2, 3,
		4, 5, 6,
		7, 8, 9;

	std::cout << "Eigen Matrix:\n" << m << std::endl << std::endl;

	// -- GLM Verification --
	glm::vec3 myVector(1.0f, 2.0f, 3.0f);

	std::cout << "GLM Vector:\n" << glm::to_string(myVector) << std::endl;
	return 0;
}