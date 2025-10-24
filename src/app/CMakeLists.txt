# Defines the RoboDesigner GUI application executable
add_executable(RoboDesignerApp
  main.cpp
 "windows/MainWindow.h" "windows/MainWindow.cpp" "widgets/RobotControlPanel.h" "widgets/RobotControlPanel.cpp" "widgets/VisualizationWidget.h" "widgets/VisualizationWidget.cpp")

# Link the application against our core library and the Qt Widgets module.
# Note the one-way dependency: App -> Core
target_link_libraries(RoboDesignerApp PRIVATE
  RoboDesignerCore
  Qt6::Widgets
)

# --- Runtime Deployment (Qt 6) ---
if(WIN32)
  # Let Qt add the appropriate deployment step for Windows (uses windeployqt internally).
  qt_finalize_executable(RoboDesignerApp)
endif()