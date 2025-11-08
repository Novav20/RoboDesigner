#pragma once

#include <vector>
#include <Eigen/Dense>

/**
 * @class IInverseKinematicsSolver
 * @brief An interface for inverse kinematics solvers.
 *
 * This abstract base class defines the contract for any class that can solve
 * the inverse kinematics problem for a robotic manipulator. It follows the
 * Strategy design pattern, allowing different IK algorithms (e.g., analytical,
 * numerical) to be used interchangeably.
 */
class IInverseKinematicsSolver {
public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IInverseKinematicsSolver() = default;

    /**
    * @brief Solves the inverse kinematics for a desired end-effector position.
    * @param target_position The desired 3D position (x, y, z) of the end-effector.
    * @param solution_index The index of the desired solution from the set of possible solutions (e.g., 0 for "elbow up", 1 for "elbow down").
    * @return A vector of the calculated joint angles. Returns an empty vector if the solution does not exist.
    */
    virtual std::vector<double> solve(const Eigen::Vector3d& target_position, int solution_index = 0) const = 0;
};