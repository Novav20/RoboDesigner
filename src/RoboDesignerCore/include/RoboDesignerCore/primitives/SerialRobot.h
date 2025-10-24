#pragma once

#include <string>
#include <vector>
#include <memory>
#include <RoboDesignerCore/kinematics/IKinematicsEngine.h>

/**
 * @class SerialRobot
 * @brief Represents a complete, controllable serial robot entity.
 *
 * This class acts as the primary public interface for a robot. It follows the
 * Strategy design pattern by owning a polymorphic IKinematicsEngine object,
 * to which it delegates all kinematics calculations. This decouples the robot
 * entity from the specific algorithm used to solve its motion, allowing for
 * greater flexibility and scalability.
 */
class SerialRobot {
public:
	/**
	 * @brief Constructs a new SerialRobot.
	 * @param name The identifying name for the robot.
	 * @param kinematics_engine A unique_ptr to a kinematics engine. The SerialRobot
	 *                          takes ownership of this engine.
	 */
	explicit SerialRobot(const std::string& name, std::unique_ptr<IKinematicsEngine> kinematics_engine);

	/**
	 * @brief Calculates the pose of the end-effector for a given set of joint positions.
	 * @param joint_positions An ordered vector of the current joint variable values.
	 * @return The 4x4 homogeneous transformation matrix of the end-effector's pose.
	 */
	Eigen::Matrix4d getEndEffectorPose(const std::vector<double>& joint_positions) const;

	/**
	 * @brief Gets the name of the robot.
	 * @return A constant reference to the robot's name string.
	 */
	const std::string& getName() const;

	size_t getNumJoints() const;

	/**
	 * @brief A factory method to create a simple 2-link planar demo robot.
	 * @return A unique_ptr to the newly created SerialRobot.
	 */
	static std::unique_ptr<SerialRobot> createTwoLinkPlanar();
private:
	std::string name_;
	std::unique_ptr<IKinematicsEngine> kinematics_engine_;
};