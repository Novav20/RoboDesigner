#include <RoboDesignerCore/primitives/SerialRobot.h>

SerialRobot::SerialRobot(const std::string& name) : name_(name)
{
}

void SerialRobot::addLink(const Link& link)
{
	links_.push_back(link);
}

size_t SerialRobot::getNumLinks() const
{
	return links_.size();
}

const std::string& SerialRobot::getName() const
{
	return name_;
}
