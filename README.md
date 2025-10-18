# RoboDesigner

A cross-platform engineering toolkit for the rapid prototyping and validation of robotic systems. Streamlines the workflow from kinematic design and trajectory generation to dynamic simulation and control analysis.

---

## 🏛️ Architecture

The project is built on a clean, layered architecture designed for modularity, testability, and scalability. It uses the **Strategy design pattern** to decouple the core robot entity from the specific algorithms used to solve its motion.

*   **`RoboDesignerCore` (Static Library):** The "Domain" layer. Contains all platform-independent robotics logic.
    *   **Primitives (`primitives/`):** Core data structures like `SerialRobot`, `Link`, and `DHParameters`. The `SerialRobot` class acts as the primary public "facade" for a robot.
    *   **Engines (`kinematics/`):** Contains the algorithmic "strategies." `IKinematicsEngine` is the abstract interface, and `SerialDH_KinematicsEngine` is the concrete implementation for D-H forward kinematics.
*   **`RoboDesignerApp` (Executable):** The main application ("UI" layer). It consumes the `RoboDesignerCore` library to build and command the robot.
*   **`RoboDesignerTests` (Executable):** A separate application for running all unit tests against the `RoboDesignerCore` library.

---

## 🛠️ Technology Stack

*   **Core Language:** C++20
*   **Build System:** CMake 3.20+
*   **GUI Framework:** Qt 6 (to be integrated)
*   **IDE:** Visual Studio 2022 (recommended)

## 📦 Core Dependencies

This project manages its dependencies automatically using CMake's `FetchContent` module.

*   **[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (v3.4.0):** A high-performance C++ library for linear algebra, used for all backend robotics calculations.
*   **[GLM](https://glm.g-truc.net/0.9.9/index.html) (v0.9.9.8):** A header-only C++ mathematics library designed for graphics software, used for 3D visualization.
*   **[GoogleTest](https://google.github.io/googletest/) (v1.14.0):** A C++ unit testing framework used to ensure code correctness and prevent regressions.

---

## 🚀 Getting Started

### Prerequisites

*   A C++20 compatible compiler (MSVC, GCC, Clang)
*   CMake (version 3.20 or newer)
*   Git
*   (Recommended) Visual Studio 2022 with the "Desktop development with C++" workload installed.

### Build and Run Instructions

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/Novav20/RoboDesigner.git
    cd RoboDesigner
    ```

2.  **Open in Visual Studio:**
    *   Launch Visual Studio 2022.
    *   Select `File > Open > Folder...` and choose the cloned `RoboDesigner` directory.
    *   Visual Studio will automatically detect the `CMakeLists.txt` and configure the project. This may take a minute as it downloads the dependencies.

3.  **Build and Run the Application:**
    *   In the top toolbar, select `RoboDesignerApp.exe` as the startup item.
    *   Press the green "Play" button (or `F5`) to build and run the main application.

4.  **Build and Run the Tests:**
    *   Build the full solution via `Build > Build Solution`.
    *   Open the Test Explorer via `Test > Test Explorer`.
    *   Click the "Run All Tests" button in the Test Explorer panel. All tests should pass.