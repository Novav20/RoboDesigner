#include "MainWindow.h"
#include "../widgets/RobotControlPanel.h"
#include <QHBoxLayout>
#include <QWidget>

MainWindow::MainWindow(QWidget* parent)
	:QMainWindow(parent)
{
	auto* central_widget = new QWidget(this);
	setCentralWidget(central_widget);
	auto* main_layout = new QHBoxLayout(central_widget);
	auto* control_panel = new RobotControlPanel();
	main_layout->addWidget(control_panel);
}
