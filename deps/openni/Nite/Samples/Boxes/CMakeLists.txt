#project(Sample-NiViewer)
set(VIEWER_EXE Boxes)

# Add all files we have for the example.
# ---------------------------------------
file(GLOB ${VIEWER_EXE}_sources ./*.cpp ./*.h ../../Include/*.h)

# Include directories
# ---------------------------------------
include_directories(${OPENNI_INCLUDE_DIR})
include_directories(${NITE_INCLUDE_DIR})
include_directories(.)

# The target for this sample
# ---------------------------------------
add_executable(${VIEWER_EXE} ${${VIEWER_EXE}_sources})

# And make it link against these libs.
# ---------------------------------------
find_library(GLUT_LIB glut)
if(DEFINED OPENNI_PLATFORM_APPLE)
	find_library(OPENGL_LIB OpenGL)
endif(DEFINED OPENNI_PLATFORM_APPLE)

target_link_libraries(
  ${VIEWER_EXE} 
  ${OPENNI_LIB_NAME}
  XnVNite
  ${GLUT_LIB} 
  ${OPENGL_LIB}
)

add_dependencies(${VIEWER_EXE} ${OPENNI_LIB_NAME})
