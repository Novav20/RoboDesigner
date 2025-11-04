#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>
#include <iostream>

SerialDH_KinematicsEngine::SerialDH_KinematicsEngine(const std::vector<Link>& links) : links_(links)
{
}

std::vector<Eigen::Matrix4d> SerialDH_KinematicsEngine::calculateAllLinkPoses(const std::vector<double>& joint_positions) const
{
	std::vector<Eigen::Matrix4d> all_poses;

	if (joint_positions.size() != links_.size()) {
		std::cerr << "Error: Number of joint positions does not match number of links." << std::endl;
		return {};
	}
	Eigen::Matrix4d pose(Eigen::Matrix4d::Identity());

	for (size_t i = 0; i < links_.size(); i++)
	{
		const Eigen::Matrix4d link_transform(links_[i].getTransformationMatrix(joint_positions[i]));
		pose = pose * link_transform;
		all_poses.push_back(pose);
	}

	return all_poses;
}


size_t SerialDH_KinematicsEngine::getNumJoints() const
{
	return links_.size();
}
