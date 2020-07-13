set(SIMULATION_SOURCE_DIR ${PROJECT_SOURCE_DIR}/projects/copter_simulation)

file(GLOB_RECURSE SIMULATION_HEADER ${SIMULATION_SOURCE_DIR}/*.h)
file(GLOB_RECURSE SIMULATION_SOURCE ${SIMULATION_SOURCE_DIR}/*.cpp)

add_library(copter_simulation STATIC ${SIMULATION_HEADER} ${SIMULATION_SOURCE})

# Includes.
target_include_directories(copter_simulation PRIVATE
  ${OPENGL_VIEWER_HEADER})
target_include_directories(copter_simulation PRIVATE
  ${PROJECT_SOURCE_DIR}/externals/tiny_obj_loader)
target_include_directories(copter_simulation PRIVATE
  ${PROJECT_SOURCE_DIR}/projects/copter_utils)
target_include_directories(copter_simulation PRIVATE
  ${SIMULATION_SOURCE_DIR})

set_target_properties(copter_simulation
  PROPERTIES
  DEBUG_POSTFIX "d"
  LINKER_LANGUAGE CXX
)

target_link_libraries(copter_simulation
  copter_utils
  tiny_obj_loader
)

# Set bigobj flags because some of our source files are large.
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()