#pragma once

#include <vector>
#include <Eigen/Dense>

/**
 * @class IJacobianSolver
 * @brief An interface for classes that can compute a robot's Jacobian matrix.
 *
 * This abstract base class defines the contract for a Jacobian solver. It decouples
 * the algorithm for computing the Jacobian from the components that use it (like
 * a numerical IK solver).
 */
class IJacobianSolver {
public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IJacobianSolver() = default;

    /**
     * @brief Calculates the geometric Jacobian for a given robot configuration.
     * @param all_link_poses A vector of the current pose of each link frame.
     * @return An Eigen::MatrixXd representing the Jacobian matrix. The size will
     *         depend on the task space and number of joints (e.g., 2xN for 2D position).
     */
    virtual Eigen::MatrixXd calculateJacobian(const std::vector<Eigen::Matrix4d>& all_link_poses) const = 0;
};