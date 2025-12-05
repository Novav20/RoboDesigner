#include <Eigen/Dense>
#include <RoboDesignerCore/dynamics/PinocchioDynamicsEngine.h>
#include <RoboDesignerCore/primitives/Link.h>
#include <gtest/gtest.h>
#include <vector>

namespace RoboDesigner::CoreTests {
TEST(PinocchioEngineTests, correctlyCalculatesTotalCenterOfMass) {
  // Arrange
  std::vector<Link> robot_links;
  Link link1({.theta_ = 0, .d_ = 0, .a_ = 1.0, .alpha_ = 0});
  link1.setMass(1.0);
  link1.setCenterOfMass({0.5, 0, 0});
  Link link2({.theta_ = 0, .d_ = 0, .a_ = 0.8, .alpha_ = 0});
  link2.setMass(0.8);
  link2.setCenterOfMass({0.4, 0, 0});
  robot_links.push_back(link1);
  robot_links.push_back(link2);

  // Act
  const PinocchioDynamicsEngine engine(robot_links);
  const Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(2);
  const auto actual_com = engine.calculateCenterOfMass(joint_positions);
  const auto expected_com = Eigen::Vector3d(0.9, 0, 0);

  // Assert
  EXPECT_NEAR(actual_com.x(), expected_com.x(), 1e-9);
}
} // namespace RoboDesigner::CoreTests