#include <RoboDesignerCore/Link.h>

Link::Link(const DHParameters& dh_params) : dh_params_(dh_params)
{
}

Eigen::Matrix4d Link::getTransformationMatrix(double joint_variable) const
{
	// TODO: Implement the actual D-H transformation matrix calculation in Week 2, Day 3.
	return Eigen::Matrix4d::Identity();
}
