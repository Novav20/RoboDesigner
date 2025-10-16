#include <gtest/gtest.h>
#include <RoboDesignerCore/primitives/SerialRobot.h>
#include <RoboDesignerCore/primitives/Link.h>

namespace RoboDesigner::CoreTests {
	TEST(SerialRobotTests, canAddLinksAndReportCorrectCount) {
		// Arrange
		SerialRobot test_robot("Test Bot");
		const Link link1({ .theta_ = 0.0, .d_ = 0.5, .a_ = 1.0, .alpha_ = 0.3 });
		const Link link2({ .theta_ = 0.4, .d_ = 0.1, .a_ = 0.0, .alpha_ = 0.2 });

		// Act
		test_robot.addLink(link1);
		test_robot.addLink(link2);

		// Assert
		EXPECT_EQ(test_robot.getNumLinks(), 2);
		EXPECT_EQ(test_robot.getName(), "Test Bot");
	}
}