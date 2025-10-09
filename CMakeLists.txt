cmake_minimum_required(VERSION 3.20)

project(RoboDesigner VERSION 1.0 LANGUAGES CXX)

# --- C++ Standard Configuration ---
set(CMAKE_CXX_STANDARD 20)

# Enforce the C++ standard must be met.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Disable compiler-specific extensions for better portability.
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Project Targets ---
# Add the main executable target for out application.
add_executable(RoboDesigner src/main.cpp)
