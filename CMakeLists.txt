cmake_minimum_required(VERSION 3.20)
project(RoboDesigner VERSION 1.0 LANGUAGES CXX)
enable_testing()

# --- Dependency Management ---
# Use FetchContent to download and prepare external libraries directly from their repositories at configure time.
# Disable building tests, examples, and docs for dependencies to speed up the build.
# Populate dependency sources and add them as subdirectories without including their install targets.
include(FetchContent)

set(BUILD_TESTING OFF CACHE BOOL "Disable building tests for all dependencies")

FetchContent_Declare(
	eigen
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG 3.4.0
)

FetchContent_Declare(
	glm
	GIT_REPOSITORY https://github.com/g-truc/glm.git
	GIT_TAG 0.9.9.8
)

FetchContent_Declare(
	googletest
	GIT_REPOSITORY https://github.com/google/googletest.git
	GIT_TAG v1.14.0
)

set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Disable Eigen tests")
set(EIGEN_BUILD_EXAMPLES OFF CACHE BOOL "Disable Eigen examples")
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen docs")
set(INSTALL_GTEST OFF CACHE BOOL "Do not install GoogleTest")

FetchContent_Populate(eigen)
FetchContent_Populate(glm)
FetchContent_Populate(googletest)

add_subdirectory(${eigen_SOURCE_DIR} ${eigen_BINARY_DIR} EXCLUDE_FROM_ALL)
add_subdirectory(${glm_SOURCE_DIR} ${glm_BINARY_DIR} EXCLUDE_FROM_ALL)
add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)

# --- C++ Standard Setup ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Project Libraries ---
# Add internal libraries before executables that depend on them.
add_subdirectory(src/RoboDesignerCore)

# --- Application Executable ---
add_executable(RoboDesignerApp src/main.cpp)
target_link_libraries(RoboDesignerApp PRIVATE RoboDesignerCore glm::glm)

# --- Test Executable ---
add_executable(RoboDesignerTests 
	tests/main_test.cpp
	"tests/core/math_utils_test.cpp"
	tests/glm_integration_test.cpp
 "tests/core/link_test.cpp" "tests/core/serial_robot_test.cpp")
target_link_libraries(RoboDesignerTests PRIVATE RoboDesignerCore gtest_main glm::glm)

include(GoogleTest)
gtest_discover_tests(RoboDesignerTests)