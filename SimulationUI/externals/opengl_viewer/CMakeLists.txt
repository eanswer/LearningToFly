# CMake entry point.
cmake_minimum_required(VERSION 3.1)

# Do not want to generate zeor_check because they cause messy folders in
# visual studio 2017.
set(CMAKE_SUPPRESS_REGENERATION true)

set(CMAKE_CXX_STANDARD 11)

set(OPENGL_VIEWER_HEADER
  "${CMAKE_CURRENT_LIST_DIR}/externals/eigen/"
  "${CMAKE_CURRENT_LIST_DIR}/externals/glew/include/"
  "${CMAKE_CURRENT_LIST_DIR}/externals/glfw/include/GLFW/"
  "${CMAKE_CURRENT_LIST_DIR}/externals/imgui/"
  "${CMAKE_CURRENT_LIST_DIR}/externals/stb/"
  "${CMAKE_CURRENT_LIST_DIR}/projects/opengl_viewer/include/"
)

# If we are at the root, give the project a name and do not expose
# OPENGL_VIEWER_HEADER.
set(IS_ROOT ${CMAKE_CURRENT_LIST_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
if(${IS_ROOT})
  project(graphics_codebase)
else(${IS_ROOT})
  set(OPENGL_VIEWER_HEADER ${OPENGL_VIEWER_HEADER} PARENT_SCOPE)
endif(${IS_ROOT})

find_package(OpenGL REQUIRED)

# Compile external dependencies.
add_subdirectory(externals)

# Expose PROJECT_SOURCE_DIR to the source code.
add_definitions(-DGRAPHICS_CODEBASE_SOURCE_DIR="${CMAKE_CURRENT_LIST_DIR}")

include_directories(${OPENGL_VIEWER_HEADER})

add_definitions(
  -DTW_STATIC
  -DTW_NO_LIB_PRAGMA
  -DTW_NO_DIRECT3D
  -DGLEW_STATIC
  -D_CRT_SECURE_NO_WARNINGS
)

# Projects.
add_subdirectory(projects/opengl_viewer/)
add_subdirectory(projects/test_opengl_viewer/)
