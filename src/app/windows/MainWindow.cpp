#include "../windows/MainWindow.h"
#include "../widgets/RobotControlPanel.h"
#include "../widgets/VisualizationWidget.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <QTimer>
#include <QSizePolicy>
#include <QStatusBar>

#include <RoboDesignerCore/primitives/SerialRobot.h>

// --- Constructor ---
MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent),
	robot_(nullptr)
{
	// --- Central Widget Setup ---
	auto* central_widget = new QWidget(this);
	setCentralWidget(central_widget);
	statusBar()->showMessage("Ready.");

	auto* main_layout = new QHBoxLayout(central_widget);
	auto* left_panel_container = new QWidget();
	auto* left_panel_layout = new QVBoxLayout(left_panel_container);

	// --- Child Widgets ---
	control_panel_ = new RobotControlPanel();
	visualization_widget_ = new VisualizationWidget();
	visualization_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

	auto* start_button = new QPushButton("Start Animation");
	auto* stop_button = new QPushButton("Stop Animation");

	// --- Timer Setup ---
	animation_timer_ = new QTimer(this);
	connect(animation_timer_, &QTimer::timeout, this, &MainWindow::runAnimationStep);

	// --- Layout Composition ---
	left_panel_layout->addWidget(control_panel_);
	left_panel_layout->addWidget(start_button);
	left_panel_layout->addWidget(stop_button);

	main_layout->addWidget(left_panel_container);
	main_layout->addWidget(visualization_widget_);

	// --- Signal-Slot Connections ---
	connect(control_panel_, &RobotControlPanel::jointParameterChanged,
		this, &MainWindow::onParameterChanged);

	connect(start_button, &QPushButton::clicked, animation_timer_, [this]() {
		animation_timer_->start(33);
		});

	connect(stop_button, &QPushButton::clicked, animation_timer_, &QTimer::stop);

	connect(control_panel_, &RobotControlPanel::solveIKRequested, this, &MainWindow::onSolveIKRequested);
}

// --- Load Robot ---
void MainWindow::loadRobot(SerialRobot* robot)
{
	robot_ = robot;

	if (robot_) {
		const size_t num_joints = robot_->getNumJoints();
		control_panel_->createJointControls(num_joints);
		joint_positions_.assign(num_joints, 0.0);
	}
	else {
		control_panel_->createJointControls(0);
	}

	updateVisualization();
}

// --- Animation Step ---
void MainWindow::runAnimationStep()
{
	joint_positions_[0] += 0.01;
	updateVisualization();
}

void MainWindow::onSolveIKRequested(double x, double y, bool use_elbow_down)
{
	if (!robot_) { return; }
	const auto solution_index = use_elbow_down ? 1 : 0;

	Eigen::Vector3d target_position(x, y, 0.0);
	const std::vector<double> solution = robot_->solveInverseKinematics(target_position, solution_index);
	if (!solution.empty()) {
		statusBar()->showMessage(
			QString("IK Solution FOUND: %1, %2").arg(solution[0]).arg(solution[1])
		);
		joint_positions_ = solution;
		updateVisualization();
	}
	else {
		statusBar()->showMessage("IK Solution not found: Target is unreachable.", 5000);
	}
}

// --- Visualization Update ---
void MainWindow::updateVisualization()
{
	if (!robot_ || joint_positions_.empty()) {
		visualization_widget_->updateRobotPoses({});
		return;
	}

	const std::vector<Eigen::Matrix4d> all_poses = robot_->getAllLinkPoses(joint_positions_);
	visualization_widget_->updateRobotPoses(all_poses);
	control_panel_->setJointPositions(joint_positions_);
}

// --- Parameter Change Handler ---
void MainWindow::onParameterChanged(int link_index, DHParameterIndex param_index, double value)
{
	if (link_index < 0 || link_index >= joint_positions_.size()) {
		return;
	}

	if (param_index == DHParameterIndex::Theta) {
		joint_positions_[link_index] = value;
		updateVisualization();
	}
}