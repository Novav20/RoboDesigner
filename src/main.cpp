#include <iostream>
#include <RoboDesignerCore/Link.h>
#include <RoboDesignerCore/SerialRobot.h>
#include <format>

int main() {
	SerialRobot my_robot("Prometheus. Serial: R-66y");
	const Link link1({ .theta_ = 0, .d_ = 0, .a_ = 1.0, .alpha_ = 0 });
	const Link link2({ .theta_ = 0, .d_ = 0, .a_ = 1.0, .alpha_ = 0 });
	my_robot.addLink(link1);
	my_robot.addLink(link2);

	std::cout << std::format("Created robot '{}' with {} links.\n",
		my_robot.getName(), my_robot.getNumLinks());

	return 0;
}