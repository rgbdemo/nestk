#project(Sample-NiUserTracker)
set(USERTRACKER_EXE Sample-NiUserTracker)

# Add all files we have for the example.
# ---------------------------------------
file(GLOB ${USERTRACKER_EXE}_sources ./*.cpp)

# Include directories
# ---------------------------------------
include_directories(${OPENNI_INCLUDE_DIR})

# The target for this sample
# ---------------------------------------
add_executable(${USERTRACKER_EXE} ${${USERTRACKER_EXE}_sources})
#add_definitions(${USERTRACKER_EXE} -pg)

#SET_TARGET_PROPERTIES(${USERTRACKER_EXE} PROPERTIES
#   PREFIX ""
#      LINK_FLAGS -pg
#      )

# And make it link against these libs.
# ---------------------------------------
find_library(GLUT_LIB glut)
if(DEFINED OPENNI_PLATFORM_APPLE)
	find_library(OPENGL_LIB OpenGL)
endif(DEFINED OPENNI_PLATFORM_APPLE)

target_link_libraries(
	${USERTRACKER_EXE}
	${OPENNI_LIB_NAME} 
	${GLUT_LIB} ${OPENGL_LIB}
)
add_dependencies(${USERTRACKER_EXE} ${OPENNI_LIB_NAME} )


