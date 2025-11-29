#include <RoboDesignerCore/kinematics/MonteCarloWorkspaceGenerator.h>
#include <RoboDesignerCore/kinematics/IForwardKinematicsSolver.h>
#include <random>
#include <numbers>

MonteCarloWorkspaceGenerator::MonteCarloWorkspaceGenerator(const IForwardKinematicsSolver& fk_solver)
	: fk_solver_(fk_solver)
{
}

std::vector<Eigen::Vector2d> MonteCarloWorkspaceGenerator::generate(int samples)
{
	std::vector<Eigen::Vector2d> points;
	points.reserve(samples); // Avoid reallocations

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(-std::numbers::pi, std::numbers::pi);

	const size_t num_joints = fk_solver_.getNumJoints();

	for (size_t i = 0; i < samples; ++i)
	{
		// 1. Create a random configuration for all joints
		std::vector<double> random_config;
		random_config.reserve(num_joints);

		for (size_t j = 0; j < num_joints; ++j)
		{
			random_config.push_back(dis(gen));
		}

		// 2. Solve FK to find where the robot ends up
		const Eigen::Matrix4d pose = fk_solver_.calculateForwardKinematics(random_config);

		// 3. Extract (x, y) and store it
		points.emplace_back(pose(0, 3), pose(1, 3));
	}
	return points;
}
