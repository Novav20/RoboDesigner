#pragma once

#include <Eigen/Dense>
#include <vector>

/**
 * @class IKinematicsEngine
 * @brief An interface for forward kinematics solvers.
 *
 * This abstract base class defines the contract for any class that can solve
 * the forward kinematics problem for a robotic manipulator. It follows the
 * Strategy design pattern, allowing different kinematic algorithms (e.g.,
 * Denavit-Hartenberg, Product of Exponentials) to be used interchangeably.
 */
class IKinematicsEngine {
public:
	/**
	 * @brief Virtual destructor.
	 * Ensures that the destructor of the derived class is called correctly
	 * when an object is deleted through a base class pointer.
	 */
	virtual ~IKinematicsEngine() = default;

	/**
	 * @brief Calculates the forward kinematics for a given set of joint positions.
	 * @param joint_positions An ordered vector of the current positions for each joint.
	 *                        For revolute joints, this is in radians.
	 *                        For prismatic joints, this is in meters.
	 * @return The resulting 4x4 homogeneous transformation matrix representing the
	 *         end-effector's pose relative to the robot's base frame.
	 */
	virtual Eigen::Matrix4d calculateForwardKinematics(const std::vector<double>& joint_positions) const {
		const auto all_poses = calculateAllLinkPoses(joint_positions);
		if (all_poses.empty()) {
			return Eigen::Matrix4d::Identity();
		}
		return all_poses.back();
	}

	/**
	* @brief Returns the number of joints/links in the kinematic chain.
	*/
	virtual size_t getNumJoints() const = 0;

	/**
	* @brief Calculates the pose of all link frames in the kinematic chain.
	* @param joint_positions An ordered vector of the current joint variable values.
	* @return An ordered vector of 4x4 pose matrices, one for each link frame,
	*         relative to the base frame. Returns an empty vector on error.
	*/
	virtual std::vector<Eigen::Matrix4d> calculateAllLinkPoses(const std::vector<double>& joint_positions) const = 0;
};