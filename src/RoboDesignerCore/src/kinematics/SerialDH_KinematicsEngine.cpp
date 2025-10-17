#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>

SerialDH_KinematicsEngine::SerialDH_KinematicsEngine(const std::vector<Link>& links): links_(links)
{
}

Eigen::Matrix4d SerialDH_KinematicsEngine::calculateForwardKinematics(const std::vector<double>& joint_positions) const
{
	// TODO: Implement the actual FK algorithm in Week 3, Day 3.
	return Eigen::Matrix4d();
}
