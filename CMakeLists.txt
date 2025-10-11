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

# --- Project Libraries (Layers) ---
# Add Core layer. This defines the RoboDesignerCore target
add_subdirectory(src/RoboDesignerCore)

# --- Application Target ---
add_executable(RoboDesignerApp src/main.cpp)

# Link our app against core library and GLM
target_link_libraries(RoboDesignerApp PRIVATE RoboDesignerCore glm::glm)

# --- Testing Target ---

# Add the test executable target for our unit tests.
add_executable(RoboDesignerTests 
tests/main_test.cpp 
"tests/eigen_integration_test.cpp"
 "tests/glm_integration_test.cpp")

# Link libraries to the RoboDesignerTests target.
target_link_libraries(RoboDesignerTests 
	PRIVATE 
		RoboDesignerCore
		gtest_main
		glm::glm
)

# Include the GoogleTest module to get helper functions.
include(GoogleTest)

# Automatically discover all tests within our executable and add them to CTest.
# This is what allows the VS Test Explorer to see our individual tests.
gtest_discover_tests(RoboDesignerTests)