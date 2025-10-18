#include <iostream>
#include <format>
#include <vector>
#include <memory>
#include <numbers>
#include <sstream>
#include <iomanip>
#include <RoboDesignerCore/primitives/SerialRobot.h>
#include <RoboDesignerCore/primitives/Link.h>
#include <RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h>

using JointPositions = std::vector<double>;

static std::string formatMatrix(const Eigen::Matrix4d& matrix) {
	std::stringstream ss;
	ss << std::fixed << std::setprecision(4) << matrix;
	return ss.str();
}

int main() {
	const std::vector<Link> robot_links = {
		Link({.theta_ = 0, .d_ = 0, .a_ = 1.0, .alpha_ = 0 }),
		Link({.theta_ = 0, .d_ = 0, .a_ = 0.8, .alpha_ = 0 })
	};
	auto kinematics_engine = std::make_unique<SerialDH_KinematicsEngine>(robot_links);
	SerialRobot my_robot("Prometheus. Serial: R-66y", std::move(kinematics_engine));

	JointPositions current_state = { 0.0, 0.0 };
	Eigen::Matrix4d pose = my_robot.getEndEffectorPose(current_state);
	std::cout << std::format("Created robot '{}'.\nRobot's pose at (0,0):\n{}\n\n", my_robot.getName(), formatMatrix(pose));

	current_state = { std::numbers::pi / 2, 0.0 };
	pose = my_robot.getEndEffectorPose(current_state);
	std::cout << std::format("Robot moved to (pi/2,0). New pose:\n{}\n\n", formatMatrix(pose));

	return 0;
}