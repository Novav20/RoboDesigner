#include "RobotControlPanel.h"
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QDoubleSpinBox>

RobotControlPanel::RobotControlPanel(QWidget* parent)
	: QWidget(parent)
{
	auto* main_layout = new QVBoxLayout(this);
	auto* link1_grup_box = new QGroupBox("Link 1 D-H Parameters");
	auto* grid_layout = new QGridLayout();
	grid_layout->addWidget(new QLabel("Theta (rad):"), 0, 0);
	theta_spin_box_ = new QDoubleSpinBox();
	grid_layout->addWidget(theta_spin_box_, 0, 1);
	
	grid_layout->addWidget(new QLabel("d (m)"), 1, 0);
	grid_layout->addWidget(new QDoubleSpinBox(), 1, 1);
	
	grid_layout->addWidget(new QLabel("a (m)"), 2, 0);
	grid_layout->addWidget(new QDoubleSpinBox(), 2, 1);

	grid_layout->addWidget(new QLabel("Alpha (rad)"), 3, 0);
	grid_layout->addWidget(new QDoubleSpinBox(), 3, 1);

	link1_grup_box->setLayout(grid_layout);

	main_layout->addWidget(link1_grup_box);
	main_layout->addStretch();
}


