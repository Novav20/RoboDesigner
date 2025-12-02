cmake_minimum_required(VERSION 3.20)
project(RoboDesigner VERSION 1.0 LANGUAGES CXX)
enable_testing()

# --- Conda Environment Setup ---
# Tell CMake to search for packages in the Conda environment FIRST
# This must be done before any find_package() calls
if(DEFINED ENV{CONDA_PREFIX})
	list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
	list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX}/lib/cmake)
	# Add conda lib directory to linker search path
	link_directories($ENV{CONDA_PREFIX}/lib)
endif()

# --- Runtime Search Path Configuration ---
# Use RPATH to embed library search paths in executables
# This allows executables to find conda libraries at runtime
set(CMAKE_BUILD_RPATH "$ENV{CONDA_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

# --- Qt Configuration ---
# Set this to ON to automatically run the MOC, UIC, and RCC compilers.
# This is essential for building Qt applications with CMake.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# --- Dependencies (System Level) ---

# 1. Qt 6
find_package(Qt6 REQUIRED COMPONENTS Widgets Concurrent)

# 2. Eigen 3
find_package(Eigen3 3.4 REQUIRED)
# Create an alias so our existing code (which links to 'eigen_interface') still works.
add_library(eigen_interface INTERFACE)
target_link_libraries(eigen_interface INTERFACE Eigen3::Eigen)

# 3. GLM
find_package(glm REQUIRED)

# 4. GoogleTest
find_package(GTest REQUIRED)

# 5. Pinocchio
find_package(pinocchio REQUIRED)

# --- C++ Standard Setup ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Project Libraries ---
# Add internal libraries before executables that depend on them.
add_subdirectory(src/RoboDesignerCore)

# --- Application Executable ---
add_subdirectory(src/app)

# --- Test Executable ---
add_executable(RoboDesignerTests 
	"tests/main_test.cpp"
	"tests/core/math_utils_test.cpp"
	"tests/glm_integration_test.cpp"
	"tests/core/link_test.cpp" 
	"tests/core/primitives/serial_robot_test.cpp" 
	"tests/core/kinematics/serial_dh_kinematics_engine_test.cpp"
 "tests/core/kinematics/two_link_planar_ik_solver_test.cpp" "tests/core/kinematics/jacobian_solver_test.cpp" 
 "tests/core/kinematics/numerical_ik_solver_test.cpp" 
 "tests/core/kinematics/workspace_generator_test.cpp")
target_link_libraries(RoboDesignerTests PRIVATE RoboDesignerCore GTest::gtest_main glm::glm)

include(GoogleTest)
gtest_discover_tests(RoboDesignerTests)