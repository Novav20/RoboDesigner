#pragma once
#include <RoboDesignerCore/kinematics/IInverseKinematicsSolver.h>
#include <cmath>

/**
 * @class TwoLinkPlanarIKSolver
 * @brief A specialized, analytical IK solver for a 2-link planar manipulator.
 *
 * This class implements the IInverseKinematicsSolver interface with a closed-form
 * geometric solution specific to a 2R robot arm. It calculates one of two
 * possible solutions (elbow up or elbow down) based on a solution index.
 */
class TwoLinkPlanarIKSolver : public IInverseKinematicsSolver
{
public:
    /**
     * @brief Constructs the 2-link planar IK solver.
     * @param link_length_1 The length of the first link (a1).
     * @param link_length_2 The length of the second link (a2).
     */
    explicit TwoLinkPlanarIKSolver(double link_length_1, double link_length_2);

    /**
     * @brief Solves the IK for the 2-link planar robot.
     * @param target_position The desired 3D position of the end-effector.
     * @param solution_index The desired solution (0 for elbow up, 1 for elbow down).
     * @return A vector containing {theta1, theta2}, or an empty vector if unreachable.
     */
    std::vector<double> solve(const Eigen::Vector3d& target_position, int solution_index = 0) const override;

private:
    double link_length_1_;
    double link_length_2_;
};