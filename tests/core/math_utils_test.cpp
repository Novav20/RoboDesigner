#include <gtest/gtest.h>
#include <RoboDesignerCore/MathUtils.h>

namespace RoboDesigner::CoreTests {
	TEST(MathUtilsTests, MatrixMultiplication) {
		// Arrange
		const Eigen::Matrix2d matrix_a = (Eigen::Matrix2d() << 1, 2, 3, 4).finished();
		const Eigen::Matrix2d matrix_b = (Eigen::Matrix2d() << 5, 6, 7, 8).finished();
		const Eigen::Matrix2d expected_result = (Eigen::Matrix2d() << 19, 22, 43, 50).finished();

		// Act
		const auto actual_result = MathUtils::multiplyMatrices(matrix_a, matrix_b);

		// Assert
		EXPECT_NEAR(actual_result(0, 0), expected_result(0, 0), 1e-9);
		EXPECT_NEAR(actual_result(0, 1), expected_result(0, 1), 1e-9);
		EXPECT_NEAR(actual_result(1, 0), expected_result(1, 0), 1e-9);
		EXPECT_NEAR(actual_result(1, 1), expected_result(1, 1), 1e-9);
	}
}