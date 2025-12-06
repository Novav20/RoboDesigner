#pragma once

#include <RoboDesignerCore/dynamics/IDynamicsEngine.h>
#include <RoboDesignerCore/primitives/Link.h>
#include <memory>
#include <pinocchio/multibody/fwd.hpp>
#include <vector>

/**
 * @class PinocchioDynamicsEngine
 * @brief A concrete implementation of IDynamicsEngine using the Pinocchio
 * library.
 *
 * This class acts as an Adapter. It converts RoboDesigner's Link structures
 * into a Pinocchio Model and translates API calls.
 */
class PinocchioDynamicsEngine : public IDynamicsEngine {
public:
  /**
   * @brief Constructs the engine by building a Pinocchio model from links.
   * @param links The kinematic chain defining the robot.
   */
  explicit PinocchioDynamicsEngine(const std::vector<Link> &links);

  // Destructor must be defined in the .cpp where the incomplete types are fully
  // defined.
  ~PinocchioDynamicsEngine() override;

  Eigen::Vector3d
  calculateCenterOfMass(const Eigen::VectorXd &joint_positions) const override;

private:
  // Pointers to the implementation objects
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;
};