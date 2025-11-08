#include <gtest/gtest.h>
#include <RoboDesignerCore/kinematics/TwoLinkPlanarIKSolver.h>
#include <RoboDesignerCore/primitives/Link.h>
#include <Eigen/Dense>
#include <vector>
#include <numbers>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>

namespace RoboDesigner::CoreTests {
	TEST(TwoLinkPlanarIKSolverTests, solvesIKForReachableTarget) {
		// Arrange
		const double link_length_1 = 1.0;
		const double link_length_2 = 0.8;
		const std::vector<Link> robot_links = { Link({0,0,link_length_1,0}), Link({0,0,link_length_2,0}) };
		const SerialDH_KinematicsEngine fk_solver(robot_links);
		const std::vector<double> initial_angles = { 0.5, -0.7 }; // e.g.
		const Eigen::Matrix4d target_pose = fk_solver.calculateForwardKinematics(initial_angles);
		const Eigen::Vector3d target_position = target_pose.block<3, 1>(0, 3);
		const TwoLinkPlanarIKSolver ik_solver(link_length_1, link_length_2);

		// Act
		const std::vector<double> final_angles = ik_solver.solve(target_position);

		// Assert
		ASSERT_EQ(final_angles.size(), 2);
		EXPECT_NEAR(final_angles[0], initial_angles[0], 1e-9);
		EXPECT_NEAR(final_angles[1], initial_angles[1], 1e-9);
	}

	TEST(TwoLinkPlanarIKSolverTests, returnsEmptyVectorForUnreachableTarget) {
		// Arrange
		const double link_length_1 = 1.0;
		const double link_length_2 = 0.8;
		const std::vector<Link> robot_links = { Link({0,0,link_length_1,0}), Link({0,0,link_length_2,0}) };
		const TwoLinkPlanarIKSolver ik_solver(link_length_1, link_length_2);
		const Eigen::Vector3d unreachable_target(100.0, 100.0, 0.0);

		// Act
		const std::vector<double> result = ik_solver.solve(unreachable_target);

		// Assert
		EXPECT_TRUE(result.empty());
	}
}