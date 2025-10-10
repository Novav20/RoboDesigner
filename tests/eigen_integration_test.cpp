// Make sure Google Test is installed and the include path is set correctly in your project settings.
// If using Visual Studio, right-click your project > Properties > C/C++ > General > Additional Include Directories.
// Add the path to the directory containing "gtest/gtest.h", e.g., C:\path\to\googletest\include

#include <gtest/gtest.h>
#include <Eigen/Dense>

TEST(EigenIntegration, MatrixMultiplication) {
	// Arrange
	const Eigen::Matrix2d matrix_a = (Eigen::Matrix2d() << 1, 2, 3, 4).finished();
	const Eigen::Matrix2d matrix_b = (Eigen::Matrix2d() << 5, 6, 7, 8).finished();
	const Eigen::Matrix2d expected_result = (Eigen::Matrix2d() << 19, 22, 43, 50).finished();

	// Act
	Eigen::Matrix2d actual_result = matrix_a * matrix_b;

	// Assert
	EXPECT_NEAR(actual_result(0, 0), expected_result(0, 0), 1e-9);
	EXPECT_NEAR(actual_result(0, 1), expected_result(0, 1), 1e-9);
	EXPECT_NEAR(actual_result(1, 0), expected_result(1, 0), 1e-9);
	EXPECT_NEAR(actual_result(1, 1), expected_result(1, 1), 1e-9);
}