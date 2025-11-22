#include <RoboDesignerCore/kinematics/JacobianTransposeIKSolver.h>
#include <RoboDesignerCore/kinematics/IForwardKinematicsSolver.h>
#include <RoboDesignerCore/kinematics/IJacobianSolver.h>
#include <Eigen/Dense>

JacobianTransposeIKSolver::JacobianTransposeIKSolver(
	std::unique_ptr<IForwardKinematicsSolver> fk_solver,
	std::unique_ptr<IJacobianSolver> jacobian_solver,
	int max_iterations,
	double tolerance,
	double alpha)
	: fk_solver_(std::move(fk_solver)), jacobian_solver_(std::move(jacobian_solver)), max_iterations_(max_iterations), tolerance_(tolerance), alpha_(alpha)
{
}

JacobianTransposeIKSolver::~JacobianTransposeIKSolver() = default;

std::vector<double> JacobianTransposeIKSolver::solve(const Eigen::Vector3d& target_position, int solution_index) const
{
	// The solution_index is ignored by this iterative solver.
	(void)solution_index;

	if (!fk_solver_ || !jacobian_solver_) {
		return {};
	}

	const size_t num_joints = fk_solver_->getNumJoints();
	if (num_joints == 0) {
		return {};
	}

	// Use an Eigen Vector for the internal calculations. It's more efficient for math.
	Eigen::VectorXd current_q = Eigen::VectorXd::Zero(num_joints);
	
	for (int i = 0; i < max_iterations_; ++i)
	{
		// 1. FK
		// Convert Eigen vector to std::vector for the FK solver interface.
		std::vector<double> q_vec(current_q.data(), current_q.data() + current_q.size());
		const auto all_link_poses = fk_solver_->calculateAllLinkPoses(q_vec);

		// 2. Error vector
		const Eigen::Vector3d p_current = all_link_poses.back().topRightCorner<3, 1>();
		const Eigen::Vector2d error = target_position.head<2>() - p_current.head<2>();

		// 3. Check for convergence
		if (error.norm() < tolerance_) {
			// Success! Convert the final Eigen vector back to a std::vector to return.
			return q_vec;
		}

		// 4. Jacobian
		const Eigen::MatrixXd jacobian = jacobian_solver_->calculateJacobian(all_link_poses);

		// 5. Calculate change in joint angles (delta_q)
		const Eigen::VectorXd delta_q = alpha_ * jacobian.transpose() * error;

		// 6. Update q
		current_q += delta_q;
	}

	// If the loop finishes, we failed to converge.
	return {};
}
