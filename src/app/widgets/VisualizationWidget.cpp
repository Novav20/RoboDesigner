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
	// Call base class first
	QFrame::paintEvent(event);

	// Create the painter
	QPainter painter(this);

	// Set the Pen
	painter.setPen(Qt::cyan);

	// Draw a Line
	painter.drawLine(0, 0, width(), height());
}
