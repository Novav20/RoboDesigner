#pragma once

#include <QMainWindow>
#include <vector>
#include <RoboDesignerCore/primitives/DHParameterIndex.h>

class SerialRobot;
class RobotControlPanel;
class VisualizationWidget;
class QTimer;

/**
 * @class MainWindow
 * @brief The main application window for RoboDesigner.
 *
 * This class serves as the top-level container for the application's UI.
 * It owns and manages the layout of the primary child widgets, such as the
 * control panel and the visualization widget, and handles the loading of
 * robot models into the UI.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    /**
     * @brief Constructs the main application window.
     * @param parent The optional parent widget.
     */
    explicit MainWindow(QWidget* parent = nullptr);

    /**
     * @brief Loads a robot model into the UI.
     *
     * This method populates the UI with controls corresponding to the
     * structure of the provided robot model.
     * @param robot A non-owning pointer to the robot to be loaded.
     */
    void loadRobot(SerialRobot* robot);

private slots:
    /**
     * @brief Slot that receives notifications when a robot parameter is changed in the UI.
     * @param link_index The zero-based index of the link being modified.
     * @param param_index The index of the D-H parameter being modified (0=theta, 1=d, etc.).
     * @param value The new value of the parameter.
     */
    void onParameterChanged(int link_index, DHParameterIndex param_index, double value);

    /**
     * @brief Advances the simple animation timer one step and refreshes the view.
     */
    void runAnimationStep();

    /**
     * @brief Slot that receives the request to perform an IK solve from the UI.
     *
     * This slot acts as the controller/adapter. It translates the UI-specific request
     * into a generic call to the backend IK solver, updates the application state,
     * and triggers a UI refresh.
     * @param x The target X-coordinate.
     * @param y The target Y-coordinate.
     * @param use_elbow_down Flag indicating if the alternate solution should be used.
     */
    void onSolveIKRequested(double x, double y, bool use_elbow_down);

    /**
     * @brief Handles updates to the IK target from the visualization widget.
     */
    void onIKTargetChanged(double x, double y);

private:
    // Pointers to the main UI components. These are owned by the widget hierarchy.
    RobotControlPanel* control_panel_;

    // A non-owning pointer to the currently active robot model.
    // The lifetime of the robot is managed elsewhere (e.g., in main()).
    SerialRobot* robot_;
    
    VisualizationWidget* visualization_widget_; 

    std::vector<double> joint_positions_;

    QTimer* animation_timer_;

    /**
     * @brief Updates the visualization widget with the latest robot state.
     */
    void updateVisualization();
};