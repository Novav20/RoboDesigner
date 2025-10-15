#pragma once

#include <string>
#include <vector>
#include <RoboDesignerCore/Link.h>

/**
 * @class SerialRobot
 * @brief Represents a serial kinematic chain, a collection of links.
 *
 * This class serves as a container for a robot's structural definition.
 * It manages an ordered sequence of Link objects that constitute the
 * manipulator. The class itself is stateless; it holds the robot's
 * constant geometric properties, while the robot's current configuration
 * (i.e., its joint values) is provided to its methods as needed.
 */
class SerialRobot {
public:
    /**
     * @brief Constructs a new SerialRobot with a given name.
     * @param name The identifying name for the robot.
     */
    explicit SerialRobot(const std::string& name);

    /**
     * @brief Adds a new link to the end of the kinematic chain.
     * @param link The Link object to add. Links are added in order from base to end-effector.
     */
    void addLink(const Link& link);

    /**
     * @brief Gets the number of links in the kinematic chain.
     * @return The number of links, which corresponds to the robot's degrees of freedom.
     */
    size_t getNumLinks() const;

    /**
     * @brief Gets the name of the robot.
     * @return A constant reference to the robot's name string.
     */
    const std::string& getName() const;

private:
    std::string name_;
    std::vector<Link> links_;
};