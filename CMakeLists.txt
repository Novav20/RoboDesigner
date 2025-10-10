cmake_minimum_required(VERSION 3.20)
project(RoboDesigner VERSION 1.0 LANGUAGES CXX)
enable_testing()

# --- Dependencie Management ---
include(FetchContent)

# Disable dependencies tests
set(BUILD_TESTING OFF CACHE BOOL "Disable building tests for all dependencies")

# Declare Eigen dependency.
FetchContent_Declare(
	eigen
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG 3.4.0
)

# Declare GLM dependency.
FetchContent_Declare(
	glm
	GIT_REPOSITORY https://github.com/g-truc/glm.git
	GIT_TAG 0.9.9.8
)

# Declare GoogleTest dependency.
FetchContent_Declare(
	googletest
	GIT_REPOSITORY https://github.com/google/googletest.git
	GIT_TAG v1.14.0
)

# Make dependencies available to the project.
FetchContent_MakeAvailable(eigen glm googletest)

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

# Link the GLM library to the RoboDesigner target.
target_link_libraries(RoboDesigner PRIVATE glm::glm)

# --- Testing Target ---

# Add the test executable target for our unit tests.
add_executable(RoboDesignerTests 
tests/main_test.cpp 
"tests/eigen_integration_test.cpp"
)

# Link libraries to the RoboDesignerTests target.
target_link_libraries(RoboDesignerTests 
	PRIVATE 
		gtest_main
		Eigen3::Eigen
)
