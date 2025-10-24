#include <RoboDesignerCore/primitives/SerialRobot.h>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>

SerialRobot::SerialRobot(const std::string& name, std::unique_ptr<IKinematicsEngine> kinematics_engine)
	: name_(name), kinematics_engine_(std::move(kinematics_engine))
{
}

const std::string& SerialRobot::getName() const
{
	return name_;
}

Eigen::Matrix4d SerialRobot::getEndEffectorPose(const std::vector<double>& joint_positions) const
{
	if (!kinematics_engine_) {
		return Eigen::Matrix4d::Identity();
	}
	return kinematics_engine_->calculateForwardKinematics(joint_positions);
}

size_t SerialRobot::getNumJoints() const
{
	if (!kinematics_engine_) {
		return 0;
	}
	return kinematics_engine_->getNumJoints();
}

std::unique_ptr<SerialRobot> SerialRobot::createTwoLinkPlanar() {
	const std::vector<Link> robot_links = {
		Link({.theta_ = 0, .d_ = 0, .a_ = 1.0, .alpha_ = 0}),
		Link({.theta_ = 0, .d_ = 0, .a_ = 0.8, .alpha_ = 0})
	};

	auto kinematics_engine = std::make_unique<SerialDH_KinematicsEngine>(robot_links);

	return std::make_unique<SerialRobot>("2-Link Planar Robot", std::move(kinematics_engine));
}
