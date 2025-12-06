#pragma once

#include <RoboDesignerCore/dynamics/IDynamicsEngine.h>
#include <RoboDesignerCore/kinematics/IForwardKinematicsSolver.h>
#include <RoboDesignerCore/kinematics/IInverseKinematicsSolver.h>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <optional>

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
   * @param kinematics_engine A unique_ptr to a kinematics engine. The
   * SerialRobot takes ownership of this engine.
   */
  explicit SerialRobot(
      const std::string &name,
      std::unique_ptr<IForwardKinematicsSolver> fk_solver,
      std::unique_ptr<IInverseKinematicsSolver> ik_solver = nullptr,
      std::unique_ptr<IDynamicsEngine> dynamics_engine = nullptr);

  /**
   * @brief Calculates the pose of the end-effector for a given set of joint
   * positions.
   * @param joint_positions An ordered vector of the current joint variable
   * values.
   * @return The 4x4 homogeneous transformation matrix of the end-effector's
   * pose.
   */
  Eigen::Matrix4d
  getEndEffectorPose(const std::vector<double> &joint_positions) const;

  /**
   * @brief Gets the name of the robot.
   * @return A constant reference to the robot's name string.
   */
  const std::string &getName() const;

  /**
   * @brief Gets the number of joints (degrees of freedom) of the robot.
   * @return The number of joints in the robot's kinematic chain.
   */
  size_t getNumJoints() const;

  /**
   * @brief A factory method to create a simple 2-link planar demo robot.
   * @return A unique_ptr to the newly created SerialRobot.
   */
  static std::unique_ptr<SerialRobot> createTwoLinkPlanar();

  /**
   * @brief Calculates the pose of all link frames for a given set of joint
   * positions.
   * @param joint_positions An ordered vector of the current joint variable
   * values.
   * @return An ordered vector of 4x4 pose matrices for each link.
   */
  std::vector<Eigen::Matrix4d>
  getAllLinkPoses(const std::vector<double> &joint_positions) const;

  /**
   * @brief Solves the inverse kinematics for a desired end-effector position.
   *
   * This method delegates the call to the attached IInverseKinematicsSolver.
   * @param target_position The desired 3D position of the end-effector.
   * @param solution_index The index of the desired solution.
   * @return A vector of joint angles, or an empty vector if no solver is
   * attached or no solution is found.
   */
  std::vector<double>
  solveInverseKinematics(const Eigen::Vector3d &target_position,
                         int solution_index = 0) const;
  const IForwardKinematicsSolver *getForwardKinematicsSolver() const;

  std::optional<Eigen::Vector3d> calculateCenterOfMass(const std::vector<double>& joint_positions) const;

private:
  std::string name_;
  std::unique_ptr<IForwardKinematicsSolver> fk_solver_;
  std::unique_ptr<IInverseKinematicsSolver> ik_solver_;
  std::unique_ptr<IDynamicsEngine> dynamics_engine_;
};