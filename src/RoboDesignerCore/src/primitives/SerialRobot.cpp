#include <RoboDesignerCore/primitives/SerialRobot.h>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>
#include <RoboDesignerCore/kinematics/TwoLinkPlanarIKSolver.h>
#include <RoboDesignerCore/kinematics/JacobianTransposeIKSolver.h>
#include <RoboDesignerCore/kinematics/JacobianSolver.h>
#include <iostream>

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

	// 1. Create the FK solver for the Robot
	auto fk_solver_for_robot = std::make_unique<SerialDH_KinematicsEngine>(robot_links);

	// 2. Create a SECOND, independent FK solver for the IK engine
	auto fk_solver_for_ik = std::make_unique<SerialDH_KinematicsEngine>(robot_links);

	auto jacobian_solver = std::make_unique<JacobianSolver>(robot_links);

	// 3. Give the second copy to the IK solver
	auto ik_solver = std::make_unique<JacobianTransposeIKSolver>(
		std::move(fk_solver_for_ik),
		std::move(jacobian_solver),
		500,
		1e-3,
		0.2
	);

	// 4. Give the first copy to the Robot
	return std::make_unique<SerialRobot>("2-Link Planar Robot (Numerical IK)",
		std::move(fk_solver_for_robot),
		std::move(ik_solver)
	);
}

std::vector<Eigen::Matrix4d> SerialRobot::getAllLinkPoses(const std::vector<double>& joint_positions) const
{
	if (!fk_solver_) {
		return {};
	}
	return fk_solver_->calculateAllLinkPoses(joint_positions);
}

std::vector<double> SerialRobot::solveInverseKinematics(const Eigen::Vector3d& target_position, int solution_index) const
{
	if (ik_solver_) {
		return ik_solver_->solve(target_position, solution_index);
	}
	return {};
}

const IForwardKinematicsSolver* SerialRobot::getForwardKinematicsSolver() const
{
	if (!fk_solver_) {
		return {};
	}
	return fk_solver_.get();
}


