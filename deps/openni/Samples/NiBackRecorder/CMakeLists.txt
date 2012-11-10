#project(Sample-NiBackRecorder)
set(BACKRECORDER_EXE Sample-NiBackRecorder)
# Add all files we have for the example.
# ---------------------------------------
file(GLOB ${BACKRECORDER_EXE}_sources ./*.cpp)

# Include directories
# ---------------------------------------
include_directories(${OPENNI_INCLUDE_DIR})

# The target for this sample
# ---------------------------------------
add_executable(${BACKRECORDER_EXE} ${${BACKRECORDER_EXE}_sources})

# And make it link against these libs.
# ---------------------------------------
target_link_libraries(${BACKRECORDER_EXE} ${OPENNI_LIB_NAME})

# Link to the shared libraries locally (from execution dir)
# ----------------------------------------------------------
#set_target_properties(${BACKRECORDER_EXE} PROPERTIES INSTALL_RPATH ".")

