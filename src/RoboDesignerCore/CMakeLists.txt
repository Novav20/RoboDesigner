# Defines the RoboDesignerCore static library
add_library(RoboDesignerCore STATIC
	src/MathUtils.cpp
	"include/RoboDesignerCore/MathUtils.h"
)

# "include" directory added to the include path of any target that links against this library
target_include_directories(RoboDesignerCore PUBLIC
	"${CMAKE_CURRENT_SOURCE_DIR}/include"

)

# Links this library to its own dependency, Eigen.
# PUBLIC means that any target linking to RoboDesignerCore will
# also get the Eigen include directories automatically.
target_link_libraries(RoboDesignerCore PUBLIC
	Eigen3::Eigen
)