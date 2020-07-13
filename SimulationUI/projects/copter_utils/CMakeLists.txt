set(UTILS_SOURCE_DIR ${PROJECT_SOURCE_DIR}/projects/copter_utils)

file(GLOB_RECURSE UTILS_HEADER ${UTILS_SOURCE_DIR}/*.h)
file(GLOB_RECURSE UTILS_SOURCE ${UTILS_SOURCE_DIR}/*.cpp)

add_library(copter_utils STATIC ${UTILS_HEADER} ${UTILS_SOURCE})

# Includes
target_include_directories(copter_utils PRIVATE ${OPENGL_VIEWER_HEADER})

set_target_properties(copter_utils
  PROPERTIES
  DEBUG_POSTFIX "d"
  LINKER_LANGUAGE CXX
)

# Set bigobj flags because some of our source files are large.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()