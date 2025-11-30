#include <RoboDesignerCore/kinematics/MonteCarloWorkspaceGenerator.h>
#include <RoboDesignerCore/kinematics/IForwardKinematicsSolver.h>
#include <random>
#include <numbers>

MonteCarloWorkspaceGenerator::MonteCarloWorkspaceGenerator(const IForwardKinematicsSolver& fk_solver)
	: fk_solver_(fk_solver)
{
}

WorkspaceResult MonteCarloWorkspaceGenerator::generate(int samples) {
	WorkspaceResult result;
	result.points.reserve(samples); // Avoid reallocations

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

		Eigen::Vector2d point(pose(0, 3), pose(1, 3));

		double dist = point.norm();

		if (dist > result.max_reach) result.max_reach = dist;

		result.points.push_back(point);
	}
	return result;
}