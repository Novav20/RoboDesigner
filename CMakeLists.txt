cmake_minimum_required(VERSION 3.20)
project(RoboDesigner VERSION 1.0 LANGUAGES CXX)

# --- Dependencie Management ---
include(FetchContent)

# Declare Eigen dependency.
FetchContent_Declare(
	eigen
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG 3.4.0
)

# Make Eigen available to the project.
FetchContent_MakeAvailable(eigen)

# --- C++ Standard Configuration ---
set(CMAKE_CXX_STANDARD 20)

# Enforce the C++ standard must be met.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable compiler-specific extensions for better portability.
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Project Targets ---
# Add the main executable target for out application.
add_executable(RoboDesigner src/main.cpp)

# Link the Eigen library to the RoboDesigner target.
target_link_libraries(RoboDesigner PRIVATE Eigen3::Eigen)