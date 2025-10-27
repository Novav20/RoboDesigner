#pragma once
#include <QFrame>

class QPaintEvent;

/**
 * @class VisualizationWidget
 * @brief A placeholder widget for the 2D/3D robot visualization scene.
 *
 * This class provides a visually distinct area in the UI where the robot
 * visualization will be rendered in future milestones. Currently, it is a
 * simple styled QFrame, but it serves as the architectural home for all
 * future graphics and rendering logic.
 */
class VisualizationWidget : public QFrame {
    Q_OBJECT

public:
    /**
     * @brief Constructs the visualization widget.
     * @param parent The optional parent widget.
     */
    explicit VisualizationWidget(QWidget* parent = nullptr);
protected:
    void paintEvent(QPaintEvent* event) override;
};