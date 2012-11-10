#project(Sample-NiSampleModule)
set(SAMPLEMODULE_LIB_NAME Sample-NiSampleModule)

# Add all files we have for the example.
# ---------------------------------------
file(GLOB ${SAMPLEMODULE_LIB_NAME}_sources ./*.cpp)

# Include directories
# ---------------------------------------
include_directories(${OPENNI_INCLUDE_DIR})

# The target for this sample
# ---------------------------------------
add_library(${SAMPLEMODULE_LIB_NAME} SHARED ${${SAMPLEMODULE_LIB_NAME}_sources})

# And make it link against these libs.
# ---------------------------------------
target_link_libraries(${SAMPLEMODULE_LIB_NAME} ${OPENNI_LIB_NAME})
#target_link_libraries(${SAMPLEMODULE_LIB_NAME} ${TINYXML_LIB_NAME})

