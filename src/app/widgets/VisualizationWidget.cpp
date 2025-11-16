#include "VisualizationWidget.h"
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QDebug>

VisualizationWidget::VisualizationWidget(QWidget* parent)
	:QFrame(parent),
	ik_target_(1.0, 0.5),
	is_dragging_(false),
	zoom_factor_(100.0)
{
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
	// Call base class first
	QFrame::paintEvent(event);

	// Create the painter
	QPainter painter(this);

	// Translate origin
	painter.translate(width() / 2.0, height() / 2.0);

	// Scale and flip Y-axis
	painter.scale(zoom_factor_, -zoom_factor_);

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


	// Draw the IK target marker
	QPen target_pen(Qt::red);
	target_pen.setWidth(2);
	target_pen.setCosmetic(true);
	painter.setPen(target_pen);

	const double crosshair_size = 0.05; // 5 cm in world units
	QPointF target_point(ik_target_.x(), ik_target_.y());
	painter.drawLine(target_point - QPointF(crosshair_size, 0), target_point + QPointF(crosshair_size, 0));
	painter.drawLine(target_point - QPointF(0, crosshair_size), target_point + QPointF(0, crosshair_size));
}

void VisualizationWidget::mousePressEvent(QMouseEvent* event)
{
	// Mouse position in pixel coordinates
	const QPointF world_pos = pixelToWorld(event->position());

	const Eigen::Vector2d click_vec(world_pos.x(), world_pos.y());
	const double distance = (ik_target_ - click_vec).norm();
	const double grab_radius = 0.1; // 10 cm

	if (event->button() == Qt::LeftButton && distance < grab_radius) {
		is_dragging_ = true;
	}
}

void VisualizationWidget::mouseMoveEvent(QMouseEvent* event)
{
	if (!is_dragging_) return;

	const QPointF world_pos = pixelToWorld(event->position());

	// Calculate the visible boundaries in world coordinates
	const double world_width = width() / zoom_factor_;
	const double world_height = height() / zoom_factor_;

	// Clamp the new position to be within the visible area
	double clamped_x = std::clamp(world_pos.x(), -world_width / 2.0, world_width / 2.0);
	double clamped_y = std::clamp(world_pos.y(), -world_height / 2.0, world_height / 2.0);

	// Update the ik_target_ with the clamped values
	ik_target_ = Eigen::Vector2d(clamped_x, clamped_y);

	update();

}

void VisualizationWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (event->button() == Qt::LeftButton) {
		is_dragging_ = false;
	}
}


QPointF VisualizationWidget::pixelToWorld(const QPointF& pixel_pos) const
{
	// Undo the translation
	double world_x = pixel_pos.x() - width() / 2.0;
	double world_y = pixel_pos.y() - height() / 2.0;

	// Undo the scale and Y-flip
	world_x /= zoom_factor_;
	world_y /= -zoom_factor_;

	return { world_x, world_y };
}
