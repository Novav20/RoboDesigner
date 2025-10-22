#include "VisualizationWidget.h"

VisualizationWidget::VisualizationWidget(QWidget* parent)
	:QFrame(parent) {
	setFrameStyle(QFrame::Panel | QFrame::Sunken);

	setMinimumSize(400, 300);
}