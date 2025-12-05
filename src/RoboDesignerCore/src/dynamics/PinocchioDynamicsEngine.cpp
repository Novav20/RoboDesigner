#include <RoboDesignerCore/dynamics/PinocchioDynamicsEngine.h>

// Specific Pinocchio headers needed for implementation
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>

// --- Constructor ---
PinocchioDynamicsEngine::PinocchioDynamicsEngine(const std::vector<Link> &links)
    : model_(std::make_unique<pinocchio::Model>()) {
  // 1. Initialize variables for the loop
  pinocchio::JointIndex parent_id = 0; // 0 is always the Universe
  pinocchio::SE3 placement = pinocchio::SE3::Identity();

  // 2. Iterate through each link to build the kinematic tree
  for (size_t i = 0; i < links.size(); ++i) {
    const auto &link = links[i];

    // --- A. Define Inertial Properties ---
    // Create Pinocchio Inertia object from our Link data
    pinocchio::Inertia inertia(link.getMass(), link.getCenterOfMass(),
                               link.getInertiaTensor());

    // --- B. Define Joint Type ---
    // For now, we assume all joints are Revolute-Z (Standard D-H theta)
    // In the future, we can check link.getDHParameters().type if we support
    // Prismatic.
    auto joint_model = pinocchio::JointModelRZ();

    // --- C. Calculate Placement ---
    // For the first joint (i=0), placement is relative to Universe (Identity).
    // For subsequent joints (i>0), placement is relative to the PREVIOUS joint.
    // This is defined by the static D-H parameters of the PREVIOUS link.
    if (i > 0) {
      // We use 0.0 for the joint variable because we want the STATIC offset
      // (d, a, alpha) that defines the frame location of this new joint.
      Eigen::Matrix4d static_transform =
          links[i - 1].getTransformationMatrix(0.0);
      placement = pinocchio::SE3(static_transform);
    }

    // --- D. Add Joint to Model ---
    // addJoint returns the index of the newly created joint
    parent_id = model_->addJoint(parent_id, joint_model, placement,
                                 "joint_" + std::to_string(i + 1));

    // --- E. Append Body Inertia ---
    // We attach the link's inertia to the joint we just created.
    model_->appendBodyToJoint(parent_id, inertia, pinocchio::SE3::Identity());
  }

  // 3. (Optional) Add the End Effector Frame
  // The last link's parameters define where the tool tip is relative to the
  // last joint.
  if (!links.empty()) {
    Eigen::Matrix4d last_transform = links.back().getTransformationMatrix(0.0);
    pinocchio::SE3 ee_placement(last_transform);
    model_->addFrame(pinocchio::Frame("end_effector", parent_id, 0,
                                      ee_placement, pinocchio::FIXED_JOINT));
  }

  // 4. Create Data
  // Allocate the scratchpad memory based on the completed model structure.
  data_ = std::make_unique<pinocchio::Data>(*model_);
}

// --- Destructor ---
// Must be defined here where Model and Data are fully known.
PinocchioDynamicsEngine::~PinocchioDynamicsEngine() = default;

// --- Calculate CoM ---
Eigen::Vector3d PinocchioDynamicsEngine::calculateCenterOfMass(
    const Eigen::VectorXd &joint_positions) const {
  // Ensure input size matches model
  if (joint_positions.size() != model_->nq) {
    // In a real app, log error or throw. For now, return zero.
    return Eigen::Vector3d::Zero();
  }

  // Call Pinocchio algorithm
  // This updates the 'data_' structure with the result
  pinocchio::centerOfMass(*model_, *data_, joint_positions);

  // Return the result stored in data.com[0] (Total CoM)
  return data_->com[0];
}