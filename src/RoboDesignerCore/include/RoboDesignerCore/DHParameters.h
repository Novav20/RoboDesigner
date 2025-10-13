#pragma once

/**
 * @struct DHParameters
 * @brief Represents the four classic Denavit-Hartenberg (D-H) parameters.
 *
 * This struct serves as a simple data container for the geometric parameters
 * that define the spatial relationship between two consecutive robot link frames.
 */
struct DHParameters {
    /**
     * @brief Joint Angle (theta) in radians.
     * The angle between the x_{i-1} and x_i axes, measured about the z_{i-1} axis.
     * This is the joint variable for a revolute joint.
     */
    double theta_;

    /**
     * @brief Link Offset (d) in meters.
     * The distance from the origin of frame {i-1} to the x_i axis, measured along the z_{i-1} axis.
     * This is the joint variable for a prismatic joint.
     */
    double d_;

    /**
     * @brief Link Length (a) in meters.
     * The distance from the z_{i-1} axis to the origin of frame {i}, measured along the x_i axis.
     * This represents the length of the common normal.
     */
    double a_;

    /**
     * @brief Link Twist (alpha) in radians.
     * The angle between the z_{i-1} and z_i axes, measured about the x_i axis.
     */
    double alpha_;
};