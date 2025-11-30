#pragma once
#include <vector>
#include <Eigen/Dense>
#include <RoboDesignerCore/kinematics/IForwardKinematicsSolver.h>

struct WorkspaceResult {
	std::vector<Eigen::Vector2d> points;
	double max_reach = 0.0;
};

/**
 * @class MonteCarloWorkspaceGenerator
 * @brief Generates a point cloud representing the robot's reachable workspace.
 *
 * This class uses the Monte Carlo method (random sampling) to estimate the
 * workspace. It randomly samples joint configurations within their limits
 * (currently -PI to +PI) and calculates the resulting end-effector positions.
 */
class MonteCarloWorkspaceGenerator {
public:
	/**
	 * @brief Constructs the generator.
	 * @param fk_solver Reference to the solver used to calculate poses.
	 *                  The solver MUST outlive this generator instance.
	 */
	explicit MonteCarloWorkspaceGenerator(const IForwardKinematicsSolver& fk_solver);

	/**
	 * @brief Runs the Monte Carlo simulation.
	 * @param samples The number of random configurations to generate.
	 * @return A vector of 2D points (x, y) representing reachable positions.
	 */
	WorkspaceResult generate(int samples);

private:
	const IForwardKinematicsSolver& fk_solver_;
};

