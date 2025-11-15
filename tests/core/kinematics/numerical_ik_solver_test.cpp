#include <gtest/gtest.h>
#include <memory>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>
#include <RoboDesignerCore/kinematics/JacobianTransposeIKSolver.h>
#include <RoboDesignerCore/kinematics/JacobianSolver.h>
#include <RoboDesignerCore/primitives/Link.h>

namespace RoboDesigner::CoreTests {
	TEST(JacobianTransposeIKSolverTests, convergesToTargetForReachablePose) 
	{
		// Arrange
		const double link_length_1 = 1.0;
		const double link_length_2 = 0.8;
		const std::vector<Link> robot_links = { Link({0,0,link_length_1,0}), Link({0,0,link_length_2,0}) };
		auto fk_solver = std::make_unique<SerialDH_KinematicsEngine>(robot_links);
		auto jacobian_solver = std::make_unique<JacobianSolver>(robot_links);
		JacobianTransposeIKSolver ik_solver(std::move(fk_solver), std::move(jacobian_solver), 400);
		const Eigen::Vector3d target_position(1.0, 0.5, 0.0);

		// Act 
		const std::vector<double> solution_q = ik_solver.solve(target_position);

		// Assert
		// 1. Validity
		ASSERT_FALSE(solution_q.empty());
		ASSERT_EQ(solution_q.size(), robot_links.size());

		// 2. Convergence
		const SerialDH_KinematicsEngine verification_fk_solver(robot_links);
		const auto final_pose = verification_fk_solver.calculateForwardKinematics(solution_q);
		const Eigen::Vector3d final_position = final_pose.topRightCorner<3, 1>();
		const double error = (target_position - final_position).norm();
		EXPECT_LT(error, 1e-4);

	}
}