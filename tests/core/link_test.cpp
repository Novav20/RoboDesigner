#include <gtest/gtest.h>
#include <RoboDesignerCore/Link.h>
#include <Eigen/Dense>
#include <numbers>

namespace RoboDesigner::CoreTests {
	TEST(LinkTests, correctlyCalculatesTransformForRevoluteJoint) {
		// Arrange
		const DHParameters dh_params_test = {
			.theta_ = 0.0,
			.d_ = 0.5,
			.a_ = 1.0,
			.alpha_ = std::numbers::pi / 2.0
		};
		const Link test_link(dh_params_test);
		const double joint_variable = std::numbers::pi / 2.0;
		const Eigen::Matrix4d expected_matrix = (Eigen::Matrix4d() << 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0.5, 0, 0, 0, 1).finished();

		// Act
		const Eigen::Matrix4d actual_matrix = test_link.getTransformationMatrix(joint_variable);

		// Assert
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++)
			{
				EXPECT_NEAR(actual_matrix(i, j), expected_matrix(i, j), 1e-9);
			}
		}
	}
}