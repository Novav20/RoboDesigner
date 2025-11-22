#pragma once

#include <memory>
#include <RoboDesignerCore/kinematics/IInverseKinematicsSolver.h>

// Forward declarations for dependencies
class IForwardKinematicsSolver;
class IJacobianSolver;

/**
 * @class JacobianTransposeIKSolver
 * @brief An iterative IK solver using the Jacobian Transpose method.
 *
 * This class implements the IInverseKinematicsSolver interface with a general-purpose
 * numerical algorithm. It is composed of an FK solver and a Jacobian solver, which
 * it uses to iteratively step the robot's configuration towards a desired target.
 */
class JacobianTransposeIKSolver : public IInverseKinematicsSolver {
public:
    /**
     * @brief Constructs the Jacobian Transpose IK solver.
     * @param fk_solver A unique_ptr to a forward kinematics solver. The solver takes ownership.
     * @param jacobian_solver A unique_ptr to a Jacobian solver. The solver takes ownership.
     * @param max_iterations The maximum number of iterations to perform before giving up.
     * @param tolerance The desired position error tolerance for a successful solution.
     * @param alpha The step size (learning rate) for each iteration.
     */
    explicit JacobianTransposeIKSolver(
        std::unique_ptr<IForwardKinematicsSolver> fk_solver,
        std::unique_ptr<IJacobianSolver> jacobian_solver,
        int max_iterations = 100,
        double tolerance = 1e-4,
        double alpha = 0.1);

    /**
     * @brief Destructor.
     * The destructor must be defined in the .cpp file to handle the unique_ptr
     * to forward-declared types.
     */
    ~JacobianTransposeIKSolver();

    /**
     * @brief Solves the IK for a target position using the Jacobian Transpose method.
     * @param target_position The desired 3D position of the end-effector.
     * @param solution_index This parameter is ignored by this solver as it only finds one solution.
     * @return A vector of joint angles, or an empty vector if it fails to converge.
     */
    std::vector<double> solve(const Eigen::Vector3d& target_position, int solution_index = 0) const override;

private:
    std::unique_ptr<IForwardKinematicsSolver> fk_solver_;
    std::unique_ptr<IJacobianSolver> jacobian_solver_;
    int max_iterations_;
    double tolerance_;
    double alpha_;
};