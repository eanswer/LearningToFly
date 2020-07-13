set(VIEWER_SOURCE_DIR ${PROJECT_SOURCE_DIR}/projects/design_viewer)

file(GLOB_RECURSE VIEWER_HEADER ${VIEWER_SOURCE_DIR}/*.h)
file(GLOB_RECURSE VIEWER_SOURCE ${VIEWER_SOURCE_DIR}/*.cpp)

add_executable(design_viewer ${VIEWER_HEADER} ${VIEWER_SOURCE})

# Includes.
target_include_directories(design_viewer PRIVATE ${OPENGL_VIEWER_HEADER})
target_include_directories(design_viewer PRIVATE ${PROJECT_SOURCE_DIR}/projects/copter_utils/)
target_include_directories(design_viewer PRIVATE ${VIEWER_SOURCE_DIR}/)
target_include_directories(design_viewer PRIVATE
  ${PROJECT_SOURCE_DIR}/projects/copter_simulation/)
target_include_directories(design_viewer PRIVATE
  ${PROJECT_SOURCE_DIR}/externals/tiny_obj_loader)

target_link_libraries(design_viewer
  opengl_viewer
  copter_simulation
  copter_utils
)
