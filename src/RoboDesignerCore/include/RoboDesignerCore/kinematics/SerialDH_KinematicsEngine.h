#pragma once

#include <vector>
#include <RoboDesignerCore/kinematics/IKinematicsEngine.h>
#include <RoboDesignerCore/primitives/Link.h>

/**
 * @class SerialDH_KinematicsEngine
 * @brief A concrete implementation of the IKinematicsEngine interface.
 *
 * This class calculates the forward kinematics of a serial robot using the
 * classic Denavit-Hartenberg (D-H) method. It stores a copy of the robot's
 * kinematic chain and uses it to perform the FK calculations.
 */
class SerialDH_KinematicsEngine : public IKinematicsEngine {
public:
    /**
     * @brief Constructs a D-H kinematics engine for a given kinematic chain.
     * @param links An ordered vector of the links that define the robot's structure.
     *              The engine will store a copy of this vector.
     */
    explicit SerialDH_KinematicsEngine(const std::vector<Link>& links);

    /**
     * @brief Calculates the forward kinematics using the D-H algorithm.
     * @param joint_positions An ordered vector of the current joint variable values.
     * @return The 4x4 homogeneous transformation matrix of the end-effector pose.
     */
    Eigen::Matrix4d calculateForwardKinematics(const std::vector<double>& joint_positions) const override;

    size_t getNumJoints() const override;

private:
    // A private copy of the robot's kinematic chain (its structure).
    std::vector<Link> links_;
};