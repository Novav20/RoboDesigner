#include <gtest/gtest.h>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>
#include <RoboDesignerCore/primitives/Link.h>
#include <vector>
#include <numbers>
#include <Eigen/Dense>

namespace RoboDesigner::CoreTests {
	TEST(SerialDH_KinematicsEngineTests, calculatesCorrectPoseForTwoLinkRobot) {
		// Arrange
		std::vector<Link> robot_links;
		const Link link1({ .theta_ = 0, .d_ = 0, .a_ = 1.0, .alpha_ = 0 });
		const Link link2({ .theta_ = 0, .d_ = 0, .a_ = 0.8, .alpha_ = 0 });
		robot_links.push_back(link1);
		robot_links.push_back(link2);
		const SerialDH_KinematicsEngine engine(robot_links);
		const std::vector<double> test_joint_positions = { std::numbers::pi / 2, 0 };
		const Eigen::Matrix4d expected_pose = (Eigen::Matrix4d() << 0, -1, 0, 0, 1, 0, 0, 1.8, 0, 0, 1, 0, 0, 0, 0, 1).finished();

		// Act
		const Eigen::Matrix4d actual_pose = engine.calculateForwardKinematics(test_joint_positions);

		// Assert
		for (size_t i = 0; i < 4; i++)
		{
			for (size_t j = 0; j < 4; j++) {
				EXPECT_NEAR(actual_pose(i, j), expected_pose(i, j), 1e-9);
			}
		}
	}
}