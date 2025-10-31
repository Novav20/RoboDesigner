#pragma once

#include <QWidget>
#include <vector>
#include <RoboDesignerCore/primitives/DHParameterIndex.h>

class QVBoxLayout;
class QDoubleSpinBox;
/**
 * @class RobotControlPanel
 * @brief A composite widget for displaying and editing robot parameters.
 *
 * This widget is responsible for dynamically generating a user interface
 * based on a given robot model's structure. It creates controls for each
 * link's D-H parameters and emits signals when any of these parameters
 * are changed by the user.
 */
class RobotControlPanel : public QWidget {
	Q_OBJECT

public:
	/**
	 * @brief Constructs the control panel widget.
	 * @param parent The optional parent widget.
	 */
	explicit RobotControlPanel(QWidget* parent = nullptr);

	/**
	 * @brief Generates UI controls for a given number of joints.
	 *
	 * This method clears any existing controls and programmatically creates
	 * a new set of input widgets (e.g., QDoubleSpinBox) for each of the
	 * four D-H parameters for every joint.
	 * @param num_joints The number of joints to create controls for.
	 */
	void createJointControls(int num_joints);

	/**
	* @brief Sets the values of the joint control spin boxes.
	*
	* This method programmatically updates the UI to display the provided
	* joint positions. It is the public interface for allowing an external
	* controller to update the view.
	* @param positions An ordered vector of joint positions (in radians) to display.
	*/
	void setJointPositions(const std::vector<double>& positions);

	/**
	* @brief Gets the current values from the joint position spin boxes.
	* @return An ordered vector of the current joint positions (in radians).
	*/
	std::vector<double> getJointPositions() const;

signals:
	/**
	 * @brief Emitted when any D-H parameter is changed in the UI.
	 * @param link_index The zero-based index of the link whose parameter was changed.
	 * @param param_index The index of the parameter (0=theta, 1=d, 2=a, 3=alpha).
	 * @param value The new value of the parameter.
	 */
	void jointParameterChanged(int link_index, DHParameterIndex param_index, double value);

private:
	// The main vertical layout that holds all the joint group boxes.
	QVBoxLayout* main_layout_;

	std::vector<QDoubleSpinBox*> joint_spin_boxes_;
};