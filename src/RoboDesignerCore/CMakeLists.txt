# Define the RoboDesignerCore static library from source and header files.
add_library(RoboDesignerCore STATIC
	"include/RoboDesignerCore/utils/MathUtils.h" "src/utils/MathUtils.cpp"
	"include/RoboDesignerCore/primitives/DHParameters.h" 
	"include/RoboDesignerCore/primitives/Link.h" "src/primitives/Link.cpp"
	"include/RoboDesignerCore/primitives/SerialRobot.h" "src/primitives/SerialRobot.cpp"
	"include/RoboDesignerCore/kinematics/IKinematicsEngine.h"
 "include/RoboDesignerCore/kinematics/SerialDH_KinematicsEngine.h" "src/kinematics/SerialDH_KinematicsEngine.cpp")

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
