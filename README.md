# RoboDesigner

A cross-platform engineering toolkit for the rapid prototyping and validation of robotic systems. Streamlines the workflow from kinematic design and trajectory generation to dynamic simulation and control analysis.

---

## 🏛️ Architecture

The project follows a clean, layered architecture to enforce separation of concerns and promote modularity.

*   **`RoboDesignerCore`:** A static library containing the core, platform-independent robotics domain logic (kinematics, dynamics, etc.). It has no knowledge of the UI.
*   **`RoboDesignerApp`:** The main executable application that provides the user interface and depends on the Core library.
*   **`RoboDesignerTests`:** A separate executable for running unit tests against the `RoboDesignerCore` library.

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