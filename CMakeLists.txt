cmake_minimum_required(VERSION 3.20)
project(RoboDesigner VERSION 1.0 LANGUAGES CXX)
enable_testing()

# --- Qt Configuration ---
# Set this to ON to automatically run the MOC, UIC, and RCC compilers.
# This is essential for building Qt applications with CMake.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# The CMAKE_PREFIX_PATH tells CMake where to look for installed packages like Qt.
# IMPORTANT: Use forward slashes, even on Windows.
set(CMAKE_PREFIX_PATH "C:/Qt/6.9.1/msvc2022_64" CACHE PATH "Path to Qt installation")

# Find the Qt6 package and its required Widgets component.
# REQUIRED means CMake will fail with an error if Qt is not found.
find_package(Qt6 REQUIRED COMPONENTS Widgets)

# --- Dependency Management ---
include(FetchContent)

# Persist FetchContent downloads across build directory wipes
set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.cache/fetchcontent" CACHE PATH "Base dir for FetchContent" FORCE)
message(STATUS "FETCHCONTENT_BASE_DIR=${FETCHCONTENT_BASE_DIR}")
file(MAKE_DIRECTORY "${FETCHCONTENT_BASE_DIR}")

# --- Eigen (Header-Only Special Case) ---
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
FetchContent_Populate(eigen)
# Create our own simple INTERFACE target. We CANNOT use a "::" in the name.
add_library(eigen_interface INTERFACE)
target_include_directories(eigen_interface INTERFACE ${eigen_SOURCE_DIR})

# --- GLM & GoogleTest (Well-Behaved Dependencies) ---
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        0.9.9.8
)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(INSTALL_GTEST OFF)
set(FETCHCONTENT_ADD_SUBDIRECTORY_EXCLUDE_FROM_ALL TRUE)
FetchContent_MakeAvailable(glm googletest)

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
 "tests/core/kinematics/two_link_planar_ik_solver_test.cpp")
target_link_libraries(RoboDesignerTests PRIVATE RoboDesignerCore gtest_main glm::glm)

include(GoogleTest)
gtest_discover_tests(RoboDesignerTests)