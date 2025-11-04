#pragma once

#include <vector>
#include <RoboDesignerCore/kinematics/IForwardKinematicsSolver.h>
#include <RoboDesignerCore/primitives/Link.h>

/**
 * @class SerialDH_KinematicsEngine
 * @brief A concrete implementation of the IKinematicsEngine interface.
 *
 * This class calculates the forward kinematics of a serial robot using the
 * classic Denavit-Hartenberg (D-H) method. It stores a copy of the robot's
 * kinematic chain and uses it to perform the FK calculations.
 */
class SerialDH_KinematicsEngine : public IForwardKinematicsSolver {
public:
	/**
	 * @brief Constructs a D-H kinematics engine for a given kinematic chain.
	 * @param links An ordered vector of the links that define the robot's structure.
	 *              The engine will store a copy of this vector.
	 */
	explicit SerialDH_KinematicsEngine(const std::vector<Link>& links);

	/**
	* @brief Calculates all link poses using the D-H algorithm.
	* @param joint_positions The current joint values.
	* @return A vector of 4x4 pose matrices for each link.
	*/
	std::vector<Eigen::Matrix4d> calculateAllLinkPoses(const std::vector<double>& joint_positions) const override;

	/**
	* @brief Gets the number of joints in the engine's kinematic chain.
	* @return The number of joints.
	*/
	size_t getNumJoints() const override;

private:
	// A private copy of the robot's kinematic chain (its structure).
	std::vector<Link> links_;
};