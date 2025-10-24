#include "../windows/MainWindow.h"
#include "../widgets/RobotControlPanel.h"
#include "../widgets/VisualizationWidget.h"
#include <QHBoxLayout>
#include <QWidget>
#include <QDebug>
#include <RoboDesignerCore/primitives/SerialRobot.h>

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent),
	robot_(nullptr) // Initialize robot pointer to null
{
	// --- Central Widget and Main Layout ---
	auto* central_widget = new QWidget(this);
	setCentralWidget(central_widget);
	auto* main_layout = new QHBoxLayout(central_widget);

	// --- Create Child Widgets ---
	control_panel_ = new RobotControlPanel(); // Assign to member variable
	auto* visualization_widget = new VisualizationWidget();
	visualization_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

	// --- Add Widgets to Layout ---
	main_layout->addWidget(control_panel_);
	main_layout->addWidget(visualization_widget);

	// --- Connect Signals and Slots ---
	// Connect the control panel's public signal to our private slot.
	connect(control_panel_, &RobotControlPanel::jointParameterChanged,
		this, &MainWindow::onParameterChanged);
}

void MainWindow::loadRobot(SerialRobot* robot)
{
	robot_ = robot; // Store the pointer to the current robot
	if (robot_) {
		const size_t num_joints = robot_->getNumJoints();
		control_panel_->createJointControls(num_joints);
	}
	else {
		// If the robot is null, clear the controls.
		control_panel_->createJointControls(0);
	}
}

// Note the updated signature to match the new signal
void MainWindow::onParameterChanged(int link_index, int param_index, double value) {
	qDebug() << "Parameter changed for Link" << link_index + 1
		<< ", Param" << param_index << "to new value:" << value;
}