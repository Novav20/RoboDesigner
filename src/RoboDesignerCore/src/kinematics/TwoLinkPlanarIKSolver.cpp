#include <RoboDesignerCore/kinematics/TwoLinkPlanarIKSolver.h>

TwoLinkPlanarIKSolver::TwoLinkPlanarIKSolver(double link_length_1, double link_length_2)
	:link_length_1_(link_length_1), link_length_2_(link_length_2)
{
}

std::vector<double> TwoLinkPlanarIKSolver::solve(const Eigen::Vector3d& target_position, int solution_index) const
{
    const double x = target_position.x();
    const double y = target_position.y();

    // Use multiplication for squaring - it's faster.
    const double cos_theta2_numerator = x * x + y * y - link_length_1_ * link_length_1_ - link_length_2_ * link_length_2_;
    const double cos_theta2_denominator = 2 * link_length_1_ * link_length_2_;

    // Avoid division by zero if a link has zero length.
    if (std::abs(cos_theta2_denominator) < 1e-9) {
        return {};
    }

    double cos_theta2 = cos_theta2_numerator / cos_theta2_denominator;

    // Safety: Clamp the value to the valid range for acos to handle floating point inaccuracies.
    // If the value is outside the range, std::clamp will bring it to the nearest boundary.
    // We still check the original value to see if it was TRULY unreachable.
    if (cos_theta2 > 1.0 || cos_theta2 < -1.0) {
        return {}; // Target is physically unreachable.
    }
    cos_theta2 = std::clamp(cos_theta2, -1.0, 1.0);
    
    double sin_theta2;
    if (solution_index == 1) {
        sin_theta2 = sqrt(1.0 - cos_theta2 * cos_theta2);
    }
    else {
        sin_theta2 = -sqrt(1.0 - cos_theta2 * cos_theta2);
    }
    const double theta2 = atan2(sin_theta2, cos_theta2);
    const double theta1 = atan2(y, x) - atan2(link_length_2_ * sin_theta2, link_length_1_ + link_length_2_ * cos_theta2);

    return { theta1, theta2 };
}