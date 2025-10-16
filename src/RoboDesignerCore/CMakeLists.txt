# Define the RoboDesignerCore static library from source and header files.
add_library(RoboDesignerCore STATIC
	src/MathUtils.cpp
	"include/RoboDesignerCore/MathUtils.h"
	"include/RoboDesignerCore/DHParameters.h" 
	"include/RoboDesignerCore/Link.h" "src/Link.cpp"
	"include/RoboDesignerCore/SerialRobot.h" "src/SerialRobot.cpp"
)

# Add the "include" directory to the include path (compile-time search paths) 
# of any target that links against RoboDesignerCore.
target_include_directories(RoboDesignerCore PUBLIC
	"${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# Link RoboDesignerCore to its dependency, Eigen.
# The PUBLIC keyword ensures that both RoboDesignerCore and any target linking to it 
# will have Eigen's include directories added automatically.
target_link_libraries(RoboDesignerCore PUBLIC
	Eigen3::Eigen
)
