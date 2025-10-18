#include <gtest/gtest.h>
#include <RoboDesignerCore/primitives/SerialRobot.h>
#include <RoboDesignerCore/primitives/Link.h>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>

namespace RoboDesigner::CoreTests {

    TEST(SerialRobotTests, correctlyConstructsAndDelegatesFK) {
        // Arrange
        const std::vector<Link> robot_links = {
            Link({.theta_ = 0.0, .d_ = 0.5, .a_ = 1.0, .alpha_ = 0.3})
        };
        const std::string robot_name = "Test Robot";

        // Use std::make_unique for exception safety. Create the engine.
        auto kinematics_engine = std::make_unique<SerialDH_KinematicsEngine>(robot_links);

        // Get a raw pointer to the engine for the assert step BEFORE moving ownership.
        // This pointer is for observation only; it does not own the memory.
        const auto* engine_ptr = kinematics_engine.get();

        // Act
        // Construct the robot, moving ownership of the engine into it.
        SerialRobot test_robot(robot_name, std::move(kinematics_engine));

        // Assert
        EXPECT_EQ(test_robot.getName(), robot_name);

        // Define the test state once.
        const std::vector<double> joint_positions = { 0.0 };

        // Call the functions only ONCE and store the results.
        const Eigen::Matrix4d robot_pose = test_robot.getEndEffectorPose(joint_positions);
        const Eigen::Matrix4d engine_pose = engine_ptr->calculateForwardKinematics(joint_positions);

        // Compare the results.
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                EXPECT_NEAR(robot_pose(i, j), engine_pose(i, j), 1e-9);
            }
        }
    }

} 