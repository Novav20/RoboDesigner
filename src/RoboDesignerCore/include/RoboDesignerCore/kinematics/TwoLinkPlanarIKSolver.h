#pragma once
#include <RoboDesignerCore/kinematics/IInverseKinematicsSolver.h>
#include <cmath>

class TwoLinkPlanarIKSolver : public IInverseKinematicsSolver 
{
public:
	explicit TwoLinkPlanarIKSolver(double link_length_1, double link_length_2);
	std::vector<double> solve(const Eigen::Vector3d& target_position, int solution_index = 0) const override;
	
private:
	double link_length_1_;
	double link_length_2_;
};