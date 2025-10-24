#pragma once
#include <QWidget>
#include <QVBoxLayout>

class RobotControlPanel : public QWidget {
	Q_OBJECT
signals: void jointParameterChanged(int liknk_index, int param_indx, double value);
public:
	explicit RobotControlPanel(QWidget* parent = nullptr);
	void createJointControls(int num_joints);
private:
	QVBoxLayout* main_layout_;
};