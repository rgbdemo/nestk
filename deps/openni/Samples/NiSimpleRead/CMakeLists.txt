#project(Sample-NiSimpleRead)
set(SIMPLEREAD_EXE Sample-NiSimpleRead)

# Add all files we have for the example.
# ---------------------------------------
file(GLOB ${SIMPLEREAD_EXE}_sources ./*.cpp)

# Include directories
# ---------------------------------------
include_directories(${OPENNI_INCLUDE_DIR})

# The target for this sample
# ---------------------------------------
add_executable(${SIMPLEREAD_EXE} ${${SIMPLEREAD_EXE}_sources})

# And make it link against these libs.
# ---------------------------------------
target_link_libraries(${SIMPLEREAD_EXE} ${OPENNI_LIB_NAME})
add_dependencies(${SIMPLEREAD_EXE} ${OPENNI_LIB_NAME})

