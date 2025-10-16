#include <RoboDesignerCore/utils/MathUtils.h>

Eigen::Matrix2d MathUtils::multiplyMatrices(const Eigen::Matrix2d& a, const Eigen::Matrix2d b)
{
	return a * b;
}
