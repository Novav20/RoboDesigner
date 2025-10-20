#include <QApplication>
#include <QWidget>

int main(int argc, char *argv[]) {
	// QApplication manages GUI application-wide resources
	QApplication app(argc, argv);

	// QWidget is the base class of all user interface objects.
	QWidget window;

	// Set the default size and title of the window.
	window.resize(800, 600);
	window.setWindowTitle("RoboDesigner");
	window.show();

	// Start the Qt event loop. This call is blocking and will only
	// return when the application is quit (e.g., the window is closed).
	return app.exec();
}