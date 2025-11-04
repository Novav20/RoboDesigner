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
     * @return A vector of the calculated joint angles. Returns an empty vector
     *         if the target is unreachable or no solution is found.
     */
    virtual std::vector<double> solve(const Eigen::Vector3d& target_position) const = 0;
};