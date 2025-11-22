/**
 * @file MathUtils.h
 * @brief Collection of small, stateless math helpers used across the project.
 */
#pragma once

#include <Eigen/Dense>

/**
 * @class MathUtils
 * @brief Provides utility math operations.
 */
class MathUtils
{
public:
    /**
     * @brief Multiplies two 2x2 matrices.
     * @param a Left-hand matrix operand.
     * @param b Right-hand matrix operand.
     * @return The product a * b.
     */
	static Eigen::Matrix2d multiplyMatrices(const Eigen::Matrix2d& a, const Eigen::Matrix2d b);
};