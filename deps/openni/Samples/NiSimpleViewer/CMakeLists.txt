set(SIMPLEVIEWER_EXE Sample-NiSimpleViewer)

# Add all files we have for the example.
# ---------------------------------------
file(GLOB ${SIMPLEVIEWER_EXE}_sources ./*.cpp)

# Include directories
# ---------------------------------------
include_directories(${OPENNI_INCLUDE_DIR})

# The target for this sample
# ---------------------------------------
add_executable(${SIMPLEVIEWER_EXE} ${${SIMPLEVIEWER_EXE}_sources})

# And make it link against these libs.
# ---------------------------------------
find_library(GLUT_LIB glut)
if(DEFINED OPENNI_PLATFORM_APPLE)
	find_library(OPENGL_LIB OpenGL)
endif(DEFINED OPENNI_PLATFORM_APPLE)

target_link_libraries(${SIMPLEVIEWER_EXE} ${OPENNI_LIB_NAME} ${GLUT_LIB} ${OPENGL_LIB})
add_dependencies(${SIMPLEVIEWER_EXE} ${OPENNI_LIB_NAME})
