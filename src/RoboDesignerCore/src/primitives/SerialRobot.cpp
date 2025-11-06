#include <RoboDesignerCore/primitives/SerialRobot.h>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>
#include <RoboDesignerCore/kinematics/TwoLinkPlanarIKSolver.h>

SerialRobot::SerialRobot(const std::string& name, std::unique_ptr<IForwardKinematicsSolver> fk_solver, std::unique_ptr<IInverseKinematicsSolver> ik_solver)
	: name_(name), fk_solver_(std::move(fk_solver)), ik_solver_(std::move(ik_solver))
{
}

const std::string& SerialRobot::getName() const
{
	return name_;
}

Eigen::Matrix4d SerialRobot::getEndEffectorPose(const std::vector<double>& joint_positions) const
{
	if (!fk_solver_) {
		return Eigen::Matrix4d::Identity();
	}
	return fk_solver_->calculateForwardKinematics(joint_positions);
}

size_t SerialRobot::getNumJoints() const
{
	if (!fk_solver_) {
		return 0;
	}
	return fk_solver_->getNumJoints();
}

std::unique_ptr<SerialRobot> SerialRobot::createTwoLinkPlanar() {
	const double link_length_1 = 1.0;
	const double link_length_2 = 0.8;

	const std::vector<Link> robot_links = {
		Link({.theta_ = 0, .d_ = 0, .a_ = link_length_1, .alpha_ = 0}),
		Link({.theta_ = 0, .d_ = 0, .a_ = link_length_2, .alpha_ = 0})
	};

	auto fk_solver = std::make_unique<SerialDH_KinematicsEngine>(robot_links);
	auto ik_solver = std::make_unique<TwoLinkPlanarIKSolver>(link_length_1, link_length_2);

	return std::make_unique<SerialRobot>("2-Link Planar Robot",
		std::move(fk_solver),
		std::move(ik_solver));
}

std::vector<Eigen::Matrix4d> SerialRobot::getAllLinkPoses(const std::vector<double>& joint_positions) const
{
	if (!fk_solver_) {
		return {};
	}
	return fk_solver_->calculateAllLinkPoses(joint_positions);
}

std::vector<double> SerialRobot::solveInverseKinematics(const Eigen::Vector3d& target_position) const
{
	if (ik_solver_ ) {
		return ik_solver_->solve(target_position);
	}
	return {};
}
