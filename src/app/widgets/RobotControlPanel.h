#pragma once
#include <QWidget>
#include <QVBoxLayout>

class RobotControlPanel : public QWidget {
	Q_OBJECT
signals: void jointParameterChanged(int link_index, int param_index, double value);
public:
	explicit RobotControlPanel(QWidget* parent = nullptr);
	void createJointControls(int num_joints);
private:
	QVBoxLayout* main_layout_;
};