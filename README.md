# RoboDesigner

A cross-platform engineering toolkit for the rapid prototyping and validation of robotic systems. Streamlines the workflow from kinematic design and trajectory generation to dynamic simulation and control analysis.

---

## 🏛️ Architecture

The project is built on a clean, layered architecture designed for modularity, testability, and scalability. It uses the **Strategy design pattern** to decouple the core robot entity from the specific algorithms used to solve its motion.

*   **`RoboDesignerCore` (Static Library):** The "Domain" layer. Contains all platform-independent robotics logic.
    *   **Primitives (`primitives/`):** Core data structures like `SerialRobot`, `Link`, and `DHParameters`. The `SerialRobot` class acts as the primary public "facade" for a robot.
    *   **Engines (`kinematics/`):** Contains the algorithmic "strategies." `IKinematicsEngine` is the abstract interface, and `SerialDH_KinematicsEngine` is the concrete implementation for D-H forward kinematics.
*   **`RoboDesignerApp` (Executable):** The main GUI application ("Presentation" layer), built with the Qt 6 framework. It consumes the `RoboDesignerCore` library to build and command the robot.
*   **`RoboDesignerTests` (Executable):** A separate application for running all unit tests against the `RoboDesignerCore` library.

---

## 🛠️ Technology Stack

*   **Core Language:** C++20
*   **GUI Framework:** Qt 6.9.1
*   **Build System:** CMake 3.20+
*   **IDE:** Visual Studio 2022 (recommended)

## 📦 Core Dependencies

This project manages its dependencies automatically using CMake's `FetchContent` module.

*   **[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (v3.4.0):** High-performance C++ library for linear algebra, used for all backend robotics calculations.
*   **[GLM](https://glm.g-truc.net/0.9.9/index.html) (v0.9.9.8):** Mathematics library for graphics software, used for 3D visualization.
*   **[GoogleTest](https://google.github.io/googletest/) (v1.14.0):** C++ unit testing framework to ensure code correctness.

---

## 🚀 Getting Started

### Prerequisites

*   A C++20 compatible compiler (e.g., MSVC from Visual Studio 2022).
*   **Qt 6.9.1** (or newer) for MSVC 2022, installed via the official Qt Maintenance Tool.
*   CMake (version 3.20 or newer).
*   Git.
*   (Recommended) Visual Studio 2022 with the "Desktop development with C++" workload.

### Build and Run Instructions

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/Novav20/RoboDesigner.git
    cd RoboDesigner
    ```

2.  **Configure the Qt Path:**
    *   Open the root `CMakeLists.txt` file.
    *   Find the line `set(CMAKE_PREFIX_PATH "C:/Qt/6.9.1/msvc2022_64" ...)`
    *   **Crucially, you must update this path** to match the exact location where you installed Qt 6 on your machine.

3.  **Open in Visual Studio:**
    *   Launch Visual Studio 2022.
    *   Select `File > Open > Folder...` and choose the cloned `RoboDesigner` directory.
    *   Visual Studio will automatically configure the project.

4.  **Build and Run the Application:**
    *   In the top toolbar, select `RoboDesignerApp.exe` as the startup item.
    *   Press the green "Play" button (or `F5`) to build and run the application. A GUI window should appear.

5.  **Build and Run the Tests:**
    *   Build the full solution via `Build > Build Solution`.
    *   Open the Test Explorer via `Test > Test Explorer`.
    *   Click the "Run All Tests" button in the Test Explorer panel. All tests should pass.