#include <QApplication>
#include "windows/MainWindow.h"
#include <RoboDesignerCore/primitives/SerialRobot.h>
#include <memory>
#include <iostream>
#include <pinocchio/config.hpp>

int main(int argc, char* argv[]) {

	std::cout << "Pinocchio Version: " << PINOCCHIO_VERSION << '\n';

	// Initialize Qt
	QApplication app(argc, argv);

	// Create the main window. It will start empty.
	MainWindow window;

	// Use our new factory method to create a demo robot.
	std::unique_ptr<SerialRobot> demo_robot = SerialRobot::createTwoLinkPlanar();

	// Load the robot into the main window, which will generate the UI.
	window.loadRobot(demo_robot.get()); // Pass a raw, non-owning pointer

	window.resize(1024, 768);
	window.setWindowTitle("RoboDesigner");
	window.show();

	return app.exec();
}