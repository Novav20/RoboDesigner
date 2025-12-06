#include "RobotControlPanel.h"
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <iostream>

RobotControlPanel::RobotControlPanel(QWidget *parent) : QWidget(parent) {
  main_layout_ = new QVBoxLayout(this);

  // Create a container widget for the dynamic controls
  dynamic_controls_host_ = new QWidget();
  dynamic_controls_host_->setLayout(
      new QVBoxLayout()); // Give it its own layout

  main_layout_->addWidget(dynamic_controls_host_); // Add host to main layout

  setupUI(); // Sets up the static IK box

  main_layout_->addStretch(); // Add the final stretch
}

void RobotControlPanel::createJointControls(int num_joints) {
  // Delete the old host widget and all its children
  delete dynamic_controls_host_;
  joint_spin_boxes_.clear();

  // Create a new, empty host
  dynamic_controls_host_ = new QWidget();
  auto *dynamic_layout = new QVBoxLayout(dynamic_controls_host_);
  dynamic_layout->setContentsMargins(0, 0, 0, 0); // makes it look tighter

  // A list of D-H parameter labels for the UI.
  const QStringList param_labels = {"Theta (rad)", "d (m)", "a (m)",
                                    "Alpha (rad)"};

  for (int i = 0; i < num_joints; ++i) {
    auto *joint_group_box = new QGroupBox(QString("Link %1").arg(i + 1));
    auto *group_box_layout = new QGridLayout();

    // Create controls for all four D-H parameters.
    for (int j = 0; j < 4; ++j) {
      auto *param_label = new QLabel(param_labels[j]);
      auto *param_spin_box = new QDoubleSpinBox();

      if (j == 0) { // If this is the 'theta' parameter
        joint_spin_boxes_.push_back(param_spin_box);
      }

      param_spin_box->setRange(-1000.0, 1000.0); // A generic, wide range
      param_spin_box->setSingleStep(0.01);
      param_spin_box->setDecimals(4);

      group_box_layout->addWidget(param_label, j, 0);
      group_box_layout->addWidget(param_spin_box, j, 1);

      // --- Connecting the Dynamic Widget ---
      connect(param_spin_box, &QDoubleSpinBox::valueChanged, this,
              [this, link_index = i,
               param_index = static_cast<DHParameterIndex>(j)](double value) {
                emit jointParameterChanged(link_index, param_index, value);
              });
    }

    joint_group_box->setLayout(group_box_layout);
    dynamic_layout->addWidget(joint_group_box);
  }
  // Insert the new host widget at the top of the main layout
  main_layout_->insertWidget(0, dynamic_controls_host_);
}

void RobotControlPanel::setJointPositions(
    const std::vector<double> &positions) {
  if (positions.size() != joint_spin_boxes_.size()) {
    std::cerr << "Error: Number of provided positions does not match number of "
                 "joint positions."
              << std::endl;
    return;
  }
  for (size_t i = 0; i < joint_spin_boxes_.size(); i++) {
    QDoubleSpinBox *spin_box = joint_spin_boxes_[i];
    spin_box->setValue(positions[i]);
  }
}

std::vector<double> RobotControlPanel::getJointPositions() const {
  std::vector<double> joint_positions;
  for (auto &spin_box : joint_spin_boxes_) {
    joint_positions.push_back(spin_box->value());
  }
  return joint_positions;
}

void RobotControlPanel::setMaxReach(double reach) {
  max_reach_display_->setText(QString::number(reach, 'f', 4));
}

void RobotControlPanel::updateCenterOfMassDisplay(double x, double y,
                                                  double z) {
  com_display_x_->setText(QString::number(x, 'f', 4));
  com_display_y_->setText(QString::number(y, 'f', 4));
  com_display_z_->setText(QString::number(z, 'f', 4));
}

// TODO: Rename to a more meaningful name
void RobotControlPanel::setupUI() {
  auto *ik_group_box = new QGroupBox("Inverse Kinematics");
  auto *workspace_group_box = new QGroupBox("Workspace Analysis");
  auto *ik_layout = new QGridLayout();
  auto *workspace_layout = new QGridLayout();

  // Target x
  ik_layout->addWidget(new QLabel("Target x (m):"), 0, 0);
  target_x_spin_box_ = new QDoubleSpinBox();
  ik_layout->addWidget(target_x_spin_box_, 0, 1);

  // Target y
  ik_layout->addWidget(new QLabel("Target y (m):"), 1, 0);
  target_y_spin_box_ = new QDoubleSpinBox();
  ik_layout->addWidget(target_y_spin_box_, 1, 1);

  // Solve buttton
  solve_ik_button_ = new QPushButton("Solve IK");
  ik_layout->addWidget(solve_ik_button_, 2, 0, 1,
                       2); // span button across 2 columns

  // Workspace config and button
  samples_spin_box_ = new QSpinBox();
  samples_spin_box_->setRange(100, 100000);
  samples_spin_box_->setValue(1000);
  generate_workspace_button_ = new QPushButton("Generate Point Cloud");
  show_workspace_checkbox_ = new QCheckBox("Show Workspace");
  show_workspace_checkbox_->setChecked(true);

  max_reach_display_ = new QLineEdit();
  max_reach_display_->setReadOnly(true);

  connect(generate_workspace_button_, &QPushButton::clicked, this, [this]() {
    const int samples = samples_spin_box_->value();
    emit workspaceGenerationRequested(samples);
  });
  connect(show_workspace_checkbox_, &QCheckBox::toggled, this,
          &RobotControlPanel::showWorkspaceChanged);

  workspace_layout->addWidget(new QLabel("Samples:"), 0, 0);
  workspace_layout->addWidget(samples_spin_box_, 0, 1);
  workspace_layout->addWidget(generate_workspace_button_, 1, 0, 1,
                              2); // Span 2 columns
  workspace_layout->addWidget(show_workspace_checkbox_, 2, 0, 1, 2);
  workspace_layout->addWidget(new QLabel("Max Reach (m):"), 3, 0);
  workspace_layout->addWidget(max_reach_display_, 3, 1);
  workspace_group_box->setLayout(workspace_layout);

  // TODO: Refactor in MVP 1.1
  elbow_down_checkbox_ = new QCheckBox("Use Elbow Down Solution");
  ik_layout->addWidget(elbow_down_checkbox_, 3, 0, 1, 2);

  connect(solve_ik_button_, &QPushButton::clicked, this, [this]() {
    const double x = target_x_spin_box_->value();
    const double y = target_y_spin_box_->value();
    const bool use_elbow_down = elbow_down_checkbox_->isChecked();
    emit solveIKRequested(x, y, use_elbow_down);
  });
  target_x_spin_box_->setRange(-10.0, 10.0);
  target_y_spin_box_->setRange(-10.0, 10.0);

  auto *dynamics_group_box = new QGroupBox("Dynamics Analysis");
  auto *dynamics_layout = new QGridLayout();

  dynamics_layout->addWidget(new QLabel("CoM X (m):"), 0, 0);
  com_display_x_ = new QLineEdit("0.000");
  com_display_x_->setReadOnly(true);
  dynamics_layout->addWidget(com_display_x_, 0, 1);

  dynamics_layout->addWidget(new QLabel("CoM Y (m):"), 1, 0);
  com_display_y_ = new QLineEdit("0.000");
  com_display_y_->setReadOnly(true);
  dynamics_layout->addWidget(com_display_y_, 1, 1);

  dynamics_layout->addWidget(new QLabel("CoM Z (m):"), 2, 0);
  com_display_z_ = new QLineEdit("0.000"); // Initialize the pointer!
  com_display_z_->setReadOnly(true);
  dynamics_layout->addWidget(com_display_z_, 2, 1);

  dynamics_group_box->setLayout(dynamics_layout);

  ik_group_box->setLayout(ik_layout);
  main_layout_->addWidget(ik_group_box);
  main_layout_->addWidget(workspace_group_box);
  main_layout_->addWidget(dynamics_group_box);
}