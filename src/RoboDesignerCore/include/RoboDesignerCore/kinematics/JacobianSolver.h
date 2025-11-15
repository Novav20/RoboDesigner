#pragma once

#include "IJacobianSolver.h"
#include <vector>

class Link;

/**
 * @class JacobianSolver
 * @brief A concrete implementation of IJacobianSolver for serial manipulators.
 *
 * This class computes the geometric Jacobian for a given serial robot structure.
 * It uses the cross-product method to determine the contribution of each joint
 * to the end-effector's linear velocity.
 */
class JacobianSolver : public IJacobianSolver {
public:
    /**
     * @brief Constructs a Jacobian solver for a specific robot structure.
     * @param links The kinematic chain (vector of Link objects) of the robot.
     */
    explicit JacobianSolver(const std::vector<Link>& links);

    /**
     * @brief Calculates the geometric Jacobian for a given robot configuration.
     * @param all_link_poses A vector of the current pose of each link frame.
     * @return A 2xN Eigen::MatrixXd representing the Jacobian for 2D planar tasks.
     */
    Eigen::MatrixXd calculateJacobian(const std::vector<Eigen::Matrix4d>& all_link_poses) const override;
private:
    std::vector<Link> links_;
};