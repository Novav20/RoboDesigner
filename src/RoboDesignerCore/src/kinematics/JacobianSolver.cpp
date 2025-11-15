#include <RoboDesignerCore/kinematics/JacobianSolver.h>
#include <RoboDesignerCore/primitives/Link.h>

JacobianSolver::JacobianSolver(const std::vector<Link>& links)
    : links_(links)
{
}

Eigen::MatrixXd JacobianSolver::calculateJacobian(const std::vector<Eigen::Matrix4d>& all_link_poses) const
{
    const size_t num_joints = links_.size();
    Eigen::MatrixXd jacobian(2, num_joints);

    // Get the end-effector position vector from the last pose.
    const Eigen::Vector3d p_ee = all_link_poses.back().topRightCorner<3, 1>();

    for (size_t i = 0; i < num_joints; ++i)
    {
        Eigen::Vector3d p_i_minus_1;
        Eigen::Vector3d z_i_minus_1;

        if (i == 0) {
            // For the first joint, its frame is at the origin, rotating around the world Z.
            p_i_minus_1 = Eigen::Vector3d::Zero();
            z_i_minus_1 = Eigen::Vector3d::UnitZ();
        }
        else {
            // For other joints, get position and Z-axis from the PREVIOUS frame's pose.
            p_i_minus_1 = all_link_poses[i - 1].topRightCorner<3, 1>();
            z_i_minus_1 = all_link_poses[i - 1].block<3, 1>(0, 2);
        }

        // For now, we assume all joints are revolute.
        // The cross product gives the linear velocity contribution.
        const Eigen::Vector3d linear_velocity_contribution = z_i_minus_1.cross(p_ee - p_i_minus_1);

        // Assign the X and Y components to the current column of the Jacobian.
        jacobian.col(i) = linear_velocity_contribution.head<2>();
    }

    return jacobian;
}