#pragma once

#include <Eigen/Dense>

/**
 * @class IDynamicsEngine
 * @brief An abstract interface for robot dynamics algorithms.
 *
 * This interface defines the contract for calculating dynamic properties
 * of a robot, such as the center of mass, inertia matrix, and inverse dynamics
 * (torques). It isolates the application from the specific physics library
 * implementation (e.g., Pinocchio, RBDL).
 */
class IDynamicsEngine {
public:
  virtual ~IDynamicsEngine() = default;

  /**
   * @brief Calculates the total Center of Mass (CoM) of the robot in the base
   * frame.
   * @param joint_positions A vector of joint positions (q).
   *                        Must match the number of degrees of freedom.
   * @return The 3D position vector of the CoM relative to the base frame.
   */
  virtual Eigen::Vector3d
  calculateCenterOfMass(const Eigen::VectorXd &joint_positions) const = 0;
};