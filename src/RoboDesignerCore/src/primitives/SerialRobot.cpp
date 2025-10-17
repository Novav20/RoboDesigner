#include <RoboDesignerCore/primitives/SerialRobot.h>

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
