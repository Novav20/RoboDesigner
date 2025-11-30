#pragma once

#include <QFrame>
#include <vector>
#include <Eigen/Dense>
#include <QPointF>
#include <QPixmap>

class QPaintEvent;
class QMouseEvent;
class QWheelEvent;

/**
 * @class VisualizationWidget
 * @brief A widget for the 2D/3D robot visualization scene.
 *
 * This class provides a visually distinct area in the UI where the robot
 * visualization will be rendered. It receives a series of pose matrices and
 * is responsible for drawing a representation of the kinematic chain.
 * It serves as the architectural "View" for the robot model.
 */
class VisualizationWidget : public QFrame {
    Q_OBJECT

public:
    /**
     * @brief Constructs the visualization widget.
     * @param parent The optional parent widget.
     */
    explicit VisualizationWidget(QWidget* parent = nullptr);

    /**
     * @brief Updates the robot pose data to be drawn.
     *
     * This is the public interface for providing the widget with new robot
     * state information. Calling this method will update the internal pose
     * data and schedule a repaint to display the changes.
     * @param poses An ordered vector of 4x4 homogeneous transformation matrices,
     *              where each matrix represents the pose of a link's frame.
     */
    void updateRobotPoses(const std::vector<Eigen::Matrix4d>& poses);

    void setWorkspacePoints(const std::vector<QPointF>& points);

    void setShowWorkspace(bool show);

signals:
    /**
     * @brief Emitted when the user updates the IK target position via interaction.
     * @param x Target x-coordinate in world units (meters).
     * @param y Target y-coordinate in world units (meters).
     */
    void targetPositionChanged(double x, double y);

protected:
    /**
     * @brief Overridden paint event to perform all custom drawing.
     * @param event The paint event data.
     */
    void paintEvent(QPaintEvent* event) override;

    /**
     * @brief Handles mouse press to begin target dragging.
     */
    void mousePressEvent(QMouseEvent* event) override;
    /**
     * @brief Handles mouse move to update target while dragging.
     */
    void mouseMoveEvent(QMouseEvent* event) override;
    /**
     * @brief Handles mouse release to end target dragging.
     */
    void mouseReleaseEvent(QMouseEvent* event) override;

    void resizeEvent(QResizeEvent* event) override;

private:
    /**
     * @brief Converts a pixel coordinate to world space using current zoom.
     */
    QPointF pixelToWorld(const QPointF& pixel_pos) const;
    // Stores the current set of link poses to be rendered.
    std::vector<Eigen::Matrix4d> robot_poses_;
    // Stores the IK target position in world coordinates (meters).
    Eigen::Vector2d ik_target_;
    // Flag to track if the user is currently dragging the target.
    bool is_dragging_;
    double zoom_factor_; // Represents pixels per meter
    std::vector<QPointF> workspace_points_;
    bool show_workspace_ = true;
    QPixmap workspace_cache_;
    void redrawWorkspaceCache();
};