#include <RoboDesignerCore/Link.h>
#include <Eigen/Geometry>

Link::Link(const DHParameters& dh_params) : dh_params_(dh_params)
{
}

Eigen::Matrix4d Link::getTransformationMatrix(double joint_variable) const
{
	const double alpha = dh_params_.alpha_;
	const double a = dh_params_.a_;
	const double d = dh_params_.d_;

	//TODO: Assume revolute joint for now
	// The total angle is the fixed offset from D-H params plus the current joint variable.
	const double theta = dh_params_.theta_ + joint_variable;

	Eigen::Affine3d rot_z(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
	Eigen::Affine3d trans_z(Eigen::Translation3d(0.0, 0.0, d));
	Eigen::Affine3d trans_x(Eigen::Translation3d(a, 0.0, 0.0));
	Eigen::Affine3d rot_x(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()));

	const Eigen::Matrix4d result = (rot_z * trans_z * trans_x * rot_x).matrix();
	return result;
}
