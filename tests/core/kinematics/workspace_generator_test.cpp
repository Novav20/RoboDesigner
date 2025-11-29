#include <gtest/gtest.h>
#include <RoboDesignerCore/kinematics/MonteCarloWorkspaceGenerator.h>
#include <RoboDesignerCore/primitives/Link.h>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>
#include <Eigen/Dense>

namespace RoboDesigner::CoreTests {
	TEST(MonteCarloWorkspaceGeneratorTests, GeneratesPointsWithinReach) {
		// Arrange
		const double link_length_1 = 1.0;
		const double link_length_2 = 0.8;
		const std::vector<Link> robot_links = { Link({0,0,link_length_1,0}), Link({0,0,link_length_2,0}) };
		const SerialDH_KinematicsEngine fk_solver(robot_links);
		MonteCarloWorkspaceGenerator workspace_generator(fk_solver);

		// Act
		auto points = workspace_generator.generate(1000);

		// Assert
		// Sanity Check: Ensure we have some variation.
		bool all_zeros = true;
		for (const auto& p : points) {
			if (p.norm() > 1e-3) { // If point is not at origin (approx)
				all_zeros = false;
				break;
			}
		}
		EXPECT_FALSE(all_zeros) << "All generated points were at (0,0). Randomness might be broken.";

		EXPECT_EQ(points.size(), 1000);
		for (const Eigen::Vector2d& point : points)
		{
			double distance_from_origin = point.norm();
			EXPECT_LE(distance_from_origin, 1.8 + 1e-9);
		}
	}
}