#include "MainWindow.h"
#include "../widgets/RobotControlPanel.h"
#include "../widgets/VisualizationWidget.h"
#include <QHBoxLayout>
#include <QWidget>
#include <QDebug>

MainWindow::MainWindow(QWidget* parent)
	:QMainWindow(parent)
{
	auto* central_widget = new QWidget(this);
	setCentralWidget(central_widget);
	auto* main_layout = new QHBoxLayout(central_widget);
	
	auto* control_panel = new RobotControlPanel();
	auto* theta_spin_box = control_panel->getThetaSpinBox();
	connect(theta_spin_box, &QDoubleSpinBox::valueChanged, this, &MainWindow::onParameterChanged);

	auto* visualization_widget = new VisualizationWidget();
	visualization_widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	main_layout->addWidget(control_panel);
	main_layout->addWidget(visualization_widget);
}

void MainWindow::onParameterChanged(double value) {
	qDebug() << "A parameter value changed to: " << value;
}