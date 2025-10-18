#include <Eigen/Dense>

class MathUtils
{
public:
	static Eigen::Matrix2d multiplyMatrices(const Eigen::Matrix2d& a, const Eigen::Matrix2d b);
};