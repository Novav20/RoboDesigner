#pragma once
#include <RoboDesignerCore/primitives/DHParameters.h>
#include <Eigen/Dense>

/**
 * @class Link
 * @brief Represents a single rigid link in a serial kinematic chain.
 *
 * This class encapsulates the geometry of a robot link as defined by the
 * Denavit-Hartenberg (D-H) convention. It holds the D-H parameters and provides
 * a method to compute the homogeneous transformation matrix that defines the
 * coordinate frame of this link relative to the previous link's frame.
 * The class is generic and does not distinguish between revolute or prismatic
 * joints; that distinction is handled by the caller of getTransformationMatrix().
 */
class Link {
public:
	/**
	 * @brief Constructs a Link with a given set of D-H parameters.
	 * @param dh_params The Denavit-Hartenberg parameters describing this link's geometry.
	 */
	explicit Link(const DHParameters& dh_params);

    /**
     * @brief Calculates the homogeneous transformation matrix for this link.
     * @param joint_value The current value of the joint's variable motion
     *                   (theta for a revolute joint, d for a prismatic joint).
     * @return A 4x4 transformation matrix.
     */
    Eigen::Matrix4d getTransformationMatrix(double joint_value) const;
private:
	DHParameters dh_params_;
};