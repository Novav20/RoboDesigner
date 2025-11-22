#include <gtest/gtest.h>
#include <RoboDesignerCore/kinematics/JacobianSolver.h>
#include <RoboDesignerCore/primitives/Link.h>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>
#include <numbers>

namespace RoboDesigner::CoreTests {
	TEST(JacobianSolverTests, calculatesCorrectJacobianForTwoLinkRobot) {
		// Arrange
		const double link_length_1 = 1.0;
		const double link_length_2 = 0.8;
		const std::vector<Link> robot_links = { Link({0,0,link_length_1,0}), Link({0,0,link_length_2,0}) };
		const SerialDH_KinematicsEngine fk_solver(robot_links);
		const JacobianSolver jacobian_solver(robot_links);

		// test configuration
		const std::vector<double> joint_angles = { 0.0, std::numbers::pi / 2.0 };
		const auto all_poses = fk_solver.calculateAllLinkPoses(joint_angles);
		// Manually calculated the ground truth Jacobian
		Eigen::Matrix2d expected_jacobian;
		expected_jacobian << -0.8, -0.8,
							  1.0,  0.0;

		// Act
		const Eigen::MatrixXd actual_jacobian = jacobian_solver.calculateJacobian(all_poses);

		// Assert
		ASSERT_EQ(actual_jacobian.rows(), 2);
		ASSERT_EQ(actual_jacobian.cols(), 2);
		for (size_t i = 0; i < actual_jacobian.rows(); i++)
		{
			for (size_t j = 0; j < actual_jacobian.cols(); j++) {
				EXPECT_NEAR(actual_jacobian(i, j), expected_jacobian(i, j), 1e-9);
			}
		}
	}
}