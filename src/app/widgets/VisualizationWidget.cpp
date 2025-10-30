#include "VisualizationWidget.h"
#include <QPainter>
#include <QPaintEvent>

VisualizationWidget::VisualizationWidget(QWidget* parent)
	:QFrame(parent) {
	setFrameStyle(QFrame::Panel | QFrame::Sunken);
	setMinimumSize(400, 300);
}

void VisualizationWidget::updateRobotPoses(const std::vector<Eigen::Matrix4d>& poses)
{
	robot_poses_ = poses;
	update(); // Schedule a repaint
}

void VisualizationWidget::paintEvent(QPaintEvent* event)
{
	// Zoom level
	const double pixels_per_meter = 100.0;

	// Call base class first
	QFrame::paintEvent(event);

	// Create the painter
	QPainter painter(this);

	// Translate origin
	painter.translate(width() / 2.0, height() / 2.0);

	// Scale and flip Y-axis
	painter.scale(pixels_per_meter, -pixels_per_meter);

	QPointF previous_point(0.0, 0.0);
	
	// Pen for the links
	QPen link_pen(Qt::white);
	link_pen.setWidth(2);
	link_pen.setCosmetic(true);


	QBrush joint_brush(Qt::cyan);

	// Base joint
	painter.setPen(Qt::NoPen); // Tell the painter not to draw an outline
	painter.setBrush(joint_brush);
	painter.drawEllipse(previous_point, 0.05, 0.05);

	for (const auto& pose : robot_poses_)
	{
		QPointF current_point(pose(0, 3), pose(1, 3)); // extract translation x, y manually
		painter.setPen(link_pen);
		painter.drawLine(previous_point, current_point);
		
		painter.setPen(Qt::NoPen);
		painter.drawEllipse(current_point, 0.05, 0.05);

		previous_point = current_point;
	}
}
