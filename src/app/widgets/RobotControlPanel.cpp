#include "RobotControlPanel.h"
#include <QGroupBox>
#include <QGridLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <iostream>

RobotControlPanel::RobotControlPanel(QWidget* parent)
	: QWidget(parent)
{
	main_layout_ = new QVBoxLayout(this);
}

void RobotControlPanel::createJointControls(int num_joints)
{
	// Clear any existing widgets from the layout.
	QLayoutItem* item;
	while ((item = main_layout_->takeAt(0)) != nullptr) {
		if (item->widget()) {
			delete item->widget();
		}
		delete item;
	}

	// A list of D-H parameter labels for the UI.
	const QStringList param_labels = { "Theta (rad)", "d (m)", "a (m)", "Alpha (rad)" };

	joint_spin_boxes_.clear();

	for (int i = 0; i < num_joints; ++i) {
		auto* joint_group_box = new QGroupBox(QString("Link %1").arg(i + 1));
		auto* group_box_layout = new QGridLayout();

		// Create controls for all four D-H parameters.
		for (int j = 0; j < 4; ++j) {
			auto* param_label = new QLabel(param_labels[j]);
			auto* param_spin_box = new QDoubleSpinBox();

			if (j == 0) { // If this is the 'theta' parameter
				joint_spin_boxes_.push_back(param_spin_box);
			}

			param_spin_box->setRange(-1000.0, 1000.0); // A generic, wide range
			param_spin_box->setSingleStep(0.01);
			param_spin_box->setDecimals(4);

			group_box_layout->addWidget(param_label, j, 0);
			group_box_layout->addWidget(param_spin_box, j, 1);

			// --- Connecting the Dynamic Widget ---
			// We use a C++ lambda function to capture the context (the link index 'i'
			// and the parameter index 'j'). When the spin box's valueChanged signal
			// is emitted, this lambda will execute.
			connect(param_spin_box, &QDoubleSpinBox::valueChanged, this,
				[this, link_index = i, param_index = static_cast<DHParameterIndex>(j)](double value) {
					// Inside the lambda, we emit our own public signal, passing along
					// all the necessary context.
					emit jointParameterChanged(link_index, param_index, value);
				}
			);
		}

		joint_group_box->setLayout(group_box_layout);
		main_layout_->addWidget(joint_group_box);
	}

	main_layout_->addStretch();
}

void RobotControlPanel::setJointPositions(const std::vector<double>& positions)
{
	if (positions.size() != joint_spin_boxes_.size()) {
		std::cerr << "Error: Number of provided positions does not match number of joint positions." << std::endl;
		return;
	}
	for (size_t i = 0; i < joint_spin_boxes_.size(); i++)
	{
		QDoubleSpinBox* spin_box = joint_spin_boxes_[i];
		spin_box->setValue(positions[i]);
	}
}

std::vector<double> RobotControlPanel::getJointPositions() const
{
	std::vector<double> joint_positions;
	for (auto& spin_box : joint_spin_boxes_) {
		joint_positions.push_back(spin_box->value());
	}
	return joint_positions;
}
