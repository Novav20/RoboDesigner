#include "VisualizationWidget.h"
#include <QPainter>
#include <QPaintEvent>

VisualizationWidget::VisualizationWidget(QWidget* parent)
	:QFrame(parent) {
	setFrameStyle(QFrame::Panel | QFrame::Sunken);

	setMinimumSize(400, 300);
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
	painter.translate(width() / 2.0, height()/2.0);

	// Scale and flip Y-axis
	painter.scale(pixels_per_meter, -pixels_per_meter);

	// Set the Pen
	QPen pen(Qt::green);
	pen.setWidth(1);
	pen.setCosmetic(true);
	painter.setPen(pen);

	// Draw a Line from world origin (0,0)  to the point (1,1) meters
	painter.drawLine(QPointF(0.0, 0.0), QPointF(1.0, 1.0));
}
