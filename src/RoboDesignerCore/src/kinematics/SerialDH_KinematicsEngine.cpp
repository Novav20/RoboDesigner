#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>
#include <iostream>

SerialDH_KinematicsEngine::SerialDH_KinematicsEngine(const std::vector<Link>& links) : links_(links)
{
}

Eigen::Matrix4d SerialDH_KinematicsEngine::calculateForwardKinematics(const std::vector<double>& joint_positions) const
{
	if (joint_positions.size() != links_.size()) {
		std::cerr << "The number of joints and links are not the same" << std::endl;
		return Eigen::Matrix4d::Identity();
	}
	Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());

	for (size_t i = 0; i < links_.size(); i++)
	{
		const Eigen::Matrix4d link_transform(links_[i].getTransformationMatrix(joint_positions[i]));
		pose = pose * link_transform;
	}

	return pose;
}

size_t SerialDH_KinematicsEngine::getNumJoints() const
{
	return links_.size();
}
