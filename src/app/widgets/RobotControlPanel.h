#pragma once
#include <QWidget>
#include <QDoubleSpinBox>

class RobotControlPanel : public QWidget {
	Q_OBJECT
public:
	explicit RobotControlPanel(QWidget* parent = nullptr);
	// Temporary public getter to facilitate learning connect()
	// In a real design, this would be replaced by a public signal.
	QDoubleSpinBox* getThetaSpinBox() const { return theta_spin_box_; }
private:
	QDoubleSpinBox* theta_spin_box_;
};