#project(Sample-NiRecordSynthetic)
set(RECORDSYNTHETIC_EXE Sample-NiRecordSynthetic)

# Add all files we have for the example.
# ---------------------------------------
file(GLOB ${RECORDSYNTHETIC_EXE}_sources ./*.cpp)

# Include directories
# ---------------------------------------
include_directories(${OPENNI_INCLUDE_DIR})

# The target for this sample
# ---------------------------------------
add_executable(${RECORDSYNTHETIC_EXE} ${${RECORDSYNTHETIC_EXE}_sources})

# And make it link against these libs.
# ---------------------------------------
#find_library(GLUT_LIB glut)
#message("Found Glut: ${GLUT_LIB}")
target_link_libraries(${RECORDSYNTHETIC_EXE} ${OPENNI_LIB_NAME})
#target_link_libraries(${RECORDSYNTHETIC_EXE} ${TINYXML_LIB_NAME})
#target_link_libraries(${RECORDSYNTHETIC_EXE} glut)
add_dependencies(${RECORDSYNTHETIC_EXE} ${OPENNI_LIB_NAME})
